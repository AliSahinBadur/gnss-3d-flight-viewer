#!/usr/bin/env python3
"""Live 3D GNSS trajectory viewer for the 78-byte HYI telemetry packet."""

from __future__ import annotations

import argparse
import atexit
import math
import struct
import threading
import time
import webbrowser
from collections import deque
from dataclasses import dataclass
from typing import Optional

try:
    import serial
except ImportError:
    serial = None


def load_dashboard_dependencies() -> None:
    """Load UI packages only when the dashboard is actually started."""
    global Dash, Input, Output, State, ctx, dcc, html, go
    try:
        from dash import Dash, Input, Output, State, ctx, dcc, html
        import plotly.graph_objects as go
    except (ImportError, AttributeError) as exc:
        raise RuntimeError(
            "Dashboard dependencies are missing or incompatible. "
            "Run: python -m pip install -r requirements.txt"
        ) from exc


HEADER = b"\xFF\xFF\x54\x52"
FOOTER = b"\x0D\x0A"
PACKET_LENGTH = 78
EARTH_RADIUS_M = 6371000.0
PAYLOAD_ALTITUDE_OFFSET = 22
PAYLOAD_LATITUDE_OFFSET = 26
PAYLOAD_LONGITUDE_OFFSET = 30

DEFAULT_SERIAL_PORT = "COM9"
DEFAULT_BAUD_RATE = 19200
DEFAULT_HTTP_PORT = 8050
APP_VERSION = 2


@dataclass(frozen=True)
class TelemetryPoint:
    time_s: float
    epoch_s: float
    packet_counter: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    east_m: float
    north_m: float
    up_m: float


def packet_checksum(packet: bytes) -> int:
    """HYI checksum: sum of bytes 4..74, modulo 256."""
    return sum(packet[4:75]) & 0xFF


def parse_hyi_packet(packet: bytes) -> tuple[int, float, float, float]:
    """Return packet counter and payload GNSS position."""
    if len(packet) != PACKET_LENGTH:
        raise ValueError(f"Expected {PACKET_LENGTH} bytes, got {len(packet)}")
    if packet[:4] != HEADER:
        raise ValueError("Invalid packet header")
    if packet[-2:] != FOOTER:
        raise ValueError("Invalid packet footer")
    if packet_checksum(packet) != packet[75]:
        raise ValueError("Invalid packet checksum")

    packet_counter = packet[5]
    altitude_m = struct.unpack_from("<f", packet, PAYLOAD_ALTITUDE_OFFSET)[0]
    latitude_deg = struct.unpack_from("<f", packet, PAYLOAD_LATITUDE_OFFSET)[0]
    longitude_deg = struct.unpack_from("<f", packet, PAYLOAD_LONGITUDE_OFFSET)[0]

    values = (altitude_m, latitude_deg, longitude_deg)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("GNSS values are not finite")
    if not -90.0 <= latitude_deg <= 90.0:
        raise ValueError("Latitude is outside valid range")
    if not -180.0 <= longitude_deg <= 180.0:
        raise ValueError("Longitude is outside valid range")

    return packet_counter, latitude_deg, longitude_deg, altitude_m


class HYIPacketDecoder:
    """Incrementally recovers valid HYI packets from an arbitrary byte stream."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.rejected_frames = 0

    def feed(self, data: bytes) -> list[bytes]:
        if data:
            self.buffer.extend(data)

        packets: list[bytes] = []
        while True:
            header_index = self.buffer.find(HEADER)
            if header_index < 0:
                keep = min(len(self.buffer), len(HEADER) - 1)
                if keep:
                    del self.buffer[:-keep]
                else:
                    self.buffer.clear()
                break

            if header_index:
                del self.buffer[:header_index]

            if len(self.buffer) < PACKET_LENGTH:
                break

            candidate = bytes(self.buffer[:PACKET_LENGTH])
            if (
                candidate[-2:] == FOOTER
                and packet_checksum(candidate) == candidate[75]
            ):
                packets.append(candidate)
                del self.buffer[:PACKET_LENGTH]
            else:
                self.rejected_frames += 1
                del self.buffer[0]

        return packets


def llh_to_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    reference_latitude_deg: float,
    reference_longitude_deg: float,
    reference_altitude_m: float,
) -> tuple[float, float, float]:
    """Convert latitude/longitude/height to a local East-North-Up frame."""
    latitude_rad = math.radians(latitude_deg)
    longitude_rad = math.radians(longitude_deg)
    reference_latitude_rad = math.radians(reference_latitude_deg)
    reference_longitude_rad = math.radians(reference_longitude_deg)

    east_m = (
        EARTH_RADIUS_M
        * (longitude_rad - reference_longitude_rad)
        * math.cos(reference_latitude_rad)
    )
    north_m = EARTH_RADIUS_M * (latitude_rad - reference_latitude_rad)
    up_m = altitude_m - reference_altitude_m
    return east_m, north_m, up_m


class TelemetryStore:
    def __init__(self, max_points: int = 20_000) -> None:
        self._points: deque[TelemetryPoint] = deque(maxlen=max_points)
        self._lock = threading.Lock()
        self._reference: Optional[tuple[float, float, float]] = None
        self._start_monotonic: Optional[float] = None
        self._status = "waiting"
        self._message = "Waiting for source"
        self._accepted_packets = 0
        self._rejected_packets = 0
        self._last_receive_epoch: Optional[float] = None

    def set_status(self, status: str, message: str) -> None:
        with self._lock:
            self._status = status
            self._message = message

    def add_rejected(self, count: int = 1) -> None:
        with self._lock:
            self._rejected_packets += count

    def append(
        self,
        packet_counter: int,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
    ) -> None:
        now_monotonic = time.monotonic()
        now_epoch = time.time()

        with self._lock:
            if self._reference is None:
                self._reference = (latitude_deg, longitude_deg, altitude_m)
                self._start_monotonic = now_monotonic

            reference_lat, reference_lon, reference_alt = self._reference
            east_m, north_m, up_m = llh_to_enu(
                latitude_deg,
                longitude_deg,
                altitude_m,
                reference_lat,
                reference_lon,
                reference_alt,
            )
            point = TelemetryPoint(
                time_s=now_monotonic - (self._start_monotonic or now_monotonic),
                epoch_s=now_epoch,
                packet_counter=packet_counter,
                latitude_deg=latitude_deg,
                longitude_deg=longitude_deg,
                altitude_m=altitude_m,
                east_m=east_m,
                north_m=north_m,
                up_m=up_m,
            )
            self._points.append(point)
            self._accepted_packets += 1
            self._last_receive_epoch = now_epoch

    def clear_track(self) -> None:
        with self._lock:
            self._points.clear()
            self._reference = None
            self._start_monotonic = None
            self._message = "Track cleared; waiting for next fix"

    def snapshot(self) -> tuple[list[TelemetryPoint], dict[str, object]]:
        with self._lock:
            return list(self._points), {
                "status": self._status,
                "message": self._message,
                "accepted": self._accepted_packets,
                "rejected": self._rejected_packets,
                "last_receive_epoch": self._last_receive_epoch,
            }


class SerialReceiver(threading.Thread):
    def __init__(
        self,
        store: TelemetryStore,
        serial_port: str,
        baud_rate: int,
        reconnect_delay_s: float = 2.0,
    ) -> None:
        super().__init__(name="serial-receiver", daemon=True)
        self.store = store
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.reconnect_delay_s = reconnect_delay_s
        self.stop_event = threading.Event()
        self.serial_connection = None

    def stop(self) -> None:
        self.stop_event.set()
        connection = self.serial_connection
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass

    def run(self) -> None:
        if serial is None:
            self.store.set_status(
                "error", "pyserial is missing; run: pip install -r requirements.txt"
            )
            return

        while not self.stop_event.is_set():
            decoder = HYIPacketDecoder()
            rejected_before = 0
            try:
                self.store.set_status(
                    "waiting",
                    f"Opening {self.serial_port} at {self.baud_rate} baud",
                )
                self.serial_connection = serial.Serial(
                    self.serial_port,
                    self.baud_rate,
                    timeout=0.25,
                )
                self.store.set_status(
                    "connected",
                    f"{self.serial_port} at {self.baud_rate} baud",
                )

                while not self.stop_event.is_set():
                    waiting = self.serial_connection.in_waiting
                    chunk = self.serial_connection.read(max(1, min(waiting, 4096)))
                    if not chunk:
                        continue

                    for packet in decoder.feed(chunk):
                        try:
                            counter, lat, lon, alt = parse_hyi_packet(packet)
                            self.store.append(counter, lat, lon, alt)
                        except ValueError:
                            self.store.add_rejected()

                    rejected_now = decoder.rejected_frames
                    if rejected_now > rejected_before:
                        self.store.add_rejected(rejected_now - rejected_before)
                        rejected_before = rejected_now

            except Exception as exc:
                if not self.stop_event.is_set():
                    self.store.set_status("error", str(exc))
            finally:
                if self.serial_connection is not None:
                    try:
                        self.serial_connection.close()
                    except Exception:
                        pass
                self.serial_connection = None

            if not self.stop_event.wait(self.reconnect_delay_s):
                self.store.set_status(
                    "waiting", f"Retrying {self.serial_port}"
                )


class DemoReceiver(threading.Thread):
    """Generates a repeatable flight path without serial hardware."""

    def __init__(self, store: TelemetryStore, update_hz: float = 8.0) -> None:
        super().__init__(name="demo-receiver", daemon=True)
        self.store = store
        self.update_hz = update_hz
        self.stop_event = threading.Event()

    def stop(self) -> None:
        self.stop_event.set()

    @staticmethod
    def altitude_profile(time_s: float) -> float:
        if time_s < 8.0:
            return 12.0 * time_s * time_s
        if time_s < 25.0:
            dt = time_s - 8.0
            return 768.0 + 78.0 * dt - 2.25 * dt * dt
        dt = time_s - 25.0
        return max(0.0, 1_442.0 - 12.5 * dt)

    def run(self) -> None:
        base_lat = 40.9621053
        base_lon = 29.1294180
        base_alt = 1_400.0
        reference_lat_rad = math.radians(base_lat)
        started = time.monotonic()
        counter = 0
        self.store.set_status("demo", "Synthetic HYI flight")

        while not self.stop_event.is_set():
            flight_time = time.monotonic() - started
            east_m = 3.8 * flight_time + 11.0 * math.sin(flight_time / 8.0)
            north_m = 2.2 * flight_time + 7.0 * math.sin(flight_time / 11.0)
            relative_altitude_m = self.altitude_profile(flight_time)

            latitude_deg = base_lat + math.degrees(north_m / EARTH_RADIUS_M)
            longitude_deg = base_lon + math.degrees(
                east_m / (EARTH_RADIUS_M * math.cos(reference_lat_rad))
            )
            altitude_m = base_alt + relative_altitude_m

            self.store.append(
                counter & 0xFF,
                latitude_deg,
                longitude_deg,
                altitude_m,
            )
            counter += 1
            self.stop_event.wait(1.0 / self.update_hz)


def calculate_speed(
    points: list[TelemetryPoint],
) -> tuple[float, float]:
    if len(points) < 2:
        return 0.0, 0.0

    last = points[-1]
    comparison_index = max(0, len(points) - 8)
    previous = points[comparison_index]
    delta_time = last.time_s - previous.time_s
    if delta_time <= 0.0:
        return 0.0, 0.0

    horizontal_distance = math.hypot(
        last.east_m - previous.east_m,
        last.north_m - previous.north_m,
    )
    vertical_distance = last.up_m - previous.up_m
    return horizontal_distance / delta_time, vertical_distance / delta_time


def padded_range(values: list[float], minimum_span: float = 20.0) -> list[float]:
    if not values:
        return [-10.0, 10.0]
    low = min(values)
    high = max(values)
    span = max(high - low, minimum_span)
    padding = span * 0.12
    center = (low + high) * 0.5
    half_span = span * 0.5 + padding
    return [center - half_span, center + half_span]


FIGURE_THEMES = {
    "dark": {
        "panel": "#0d1014",
        "axis": "#11161c",
        "grid": "#303943",
        "zero": "#59636f",
        "text": "#d9dde3",
        "muted": "#aab2bd",
        "projection": "#68727d",
        "reference": "#60a5fa",
        "legend": "rgba(13,16,20,0.72)",
        "border": "#313944",
    },
    "light": {
        "panel": "#f6f8fb",
        "axis": "#ffffff",
        "grid": "#d7dee7",
        "zero": "#94a3b8",
        "text": "#1f2937",
        "muted": "#64748b",
        "projection": "#64748b",
        "reference": "#2563eb",
        "legend": "rgba(255,255,255,0.86)",
        "border": "#cbd5e1",
    },
}


def representative_trajectory_coordinates(
    sample_count: int = 121,
) -> tuple[list[float], list[float], list[float]]:
    progress = [index / (sample_count - 1) for index in range(sample_count)]
    east = [80.0 * value for value in progress]
    north = [18.0 * math.sin(math.pi * value) + 6.0 * value for value in progress]
    up = [120.0 * math.sin(math.pi * value) ** 1.25 for value in progress]
    return east, north, up


def add_representative_trajectory(figure: go.Figure, palette: dict[str, str]):
    east, north, up = representative_trajectory_coordinates()
    figure.add_trace(
        go.Scatter3d(
            x=east,
            y=north,
            z=up,
            mode="lines",
            name="Representative trajectory",
            line={
                "color": palette["reference"],
                "width": 6,
                "dash": "dash",
            },
            hovertemplate=(
                "Representative path<br>"
                "East: %{x:.1f} m<br>"
                "North: %{y:.1f} m<br>"
                "Up: %{z:.1f} m"
                "<extra></extra>"
            ),
        )
    )
    return east, north, up


def apply_trajectory_layout(
    figure: go.Figure,
    palette: dict[str, str],
    east: list[float],
    north: list[float],
    up: list[float],
) -> None:
    axis_style = {
        "showbackground": True,
        "backgroundcolor": palette["axis"],
        "gridcolor": palette["grid"],
        "zerolinecolor": palette["zero"],
        "tickfont": {"color": palette["muted"]},
        "showspikes": False,
    }

    figure.update_layout(
        uirevision="keep-camera",
        paper_bgcolor=palette["panel"],
        plot_bgcolor=palette["panel"],
        font={"family": "Inter, Segoe UI, Arial", "color": palette["text"]},
        margin={"l": 0, "r": 0, "t": 8, "b": 0},
        showlegend=True,
        legend={
            "x": 0.02,
            "y": 0.98,
            "bgcolor": palette["legend"],
            "bordercolor": palette["border"],
            "borderwidth": 1,
            "font": {"size": 11},
        },
        scene={
            "xaxis": {
                **axis_style,
                "title": {
                    "text": "East (m)",
                    "font": {"color": palette["text"]},
                },
                "range": padded_range(east),
            },
            "yaxis": {
                **axis_style,
                "title": {
                    "text": "North (m)",
                    "font": {"color": palette["text"]},
                },
                "range": padded_range(north),
            },
            "zaxis": {
                **axis_style,
                "title": {
                    "text": "Up (m)",
                    "font": {"color": palette["text"]},
                },
                "range": padded_range(up),
            },
            "aspectmode": "data",
            "camera": {
                "eye": {"x": 1.45, "y": 1.55, "z": 1.1},
                "up": {"x": 0.0, "y": 0.0, "z": 1.0},
            },
        },
    )


def create_trajectory_figure(
    points: list[TelemetryPoint],
    theme: str = "dark",
    show_reference: bool = False,
) -> go.Figure:
    palette = FIGURE_THEMES.get(theme, FIGURE_THEMES["dark"])
    figure = go.Figure()

    if not points:
        if show_reference:
            east, north, up = add_representative_trajectory(figure, palette)
            apply_trajectory_layout(figure, palette, east, north, up)
            return figure

        figure.add_annotation(
            text="Waiting for a valid GNSS packet",
            x=0.5,
            y=0.5,
            xref="paper",
            yref="paper",
            showarrow=False,
            font={"size": 18, "color": palette["muted"]},
        )
        figure.update_layout(
            paper_bgcolor=palette["panel"],
            plot_bgcolor=palette["panel"],
            font={"family": "Inter, Segoe UI, Arial", "color": palette["text"]},
            margin={"l": 0, "r": 0, "t": 0, "b": 0},
            xaxis={"visible": False},
            yaxis={"visible": False},
        )
        return figure

    max_plot_points = 3_000
    step = max(1, len(points) // max_plot_points)
    plotted = points[::step]
    if plotted[-1] is not points[-1]:
        plotted.append(points[-1])

    east = [point.east_m for point in plotted]
    north = [point.north_m for point in plotted]
    up = [point.up_m for point in plotted]
    altitude = [point.altitude_m for point in plotted]
    latitude = [point.latitude_deg for point in plotted]
    longitude = [point.longitude_deg for point in plotted]
    elapsed = [point.time_s for point in plotted]

    hover_data = [
        [lat, lon, alt, seconds]
        for lat, lon, alt, seconds in zip(
            latitude, longitude, altitude, elapsed
        )
    ]

    figure.add_trace(
        go.Scatter3d(
            x=east,
            y=north,
            z=up,
            mode="lines+markers",
            name="Flight path",
            line={"color": "#ff9f43", "width": 6},
            marker={
                "size": 3,
                "color": up,
                "colorscale": "Turbo",
                "showscale": False,
                "colorbar": {
                    "title": {
                        "text": "Up (m)",
                        "font": {"color": palette["text"]},
                    },
                    "thickness": 12,
                    "len": 0.55,
                    "x": 0.92,
                    "tickfont": {"color": palette["muted"]},
                },
            },
            customdata=hover_data,
            hovertemplate=(
                "East: %{x:.1f} m<br>"
                "North: %{y:.1f} m<br>"
                "Up: %{z:.1f} m<br>"
                "Lat: %{customdata[0]:.7f}<br>"
                "Lon: %{customdata[1]:.7f}<br>"
                "Alt: %{customdata[2]:.1f} m<br>"
                "Time: %{customdata[3]:.1f} s"
                "<extra></extra>"
            ),
        )
    )

    figure.add_trace(
        go.Scatter3d(
            x=east,
            y=north,
            z=[0.0] * len(east),
            mode="lines",
            name="Ground projection",
            line={"color": palette["projection"], "width": 3, "dash": "dot"},
            hoverinfo="skip",
        )
    )

    current = points[-1]
    figure.add_trace(
        go.Scatter3d(
            x=[current.east_m, current.east_m],
            y=[current.north_m, current.north_m],
            z=[0.0, current.up_m],
            mode="lines",
            name="Altitude",
            line={"color": "#57d3a2", "width": 3, "dash": "dash"},
            hoverinfo="skip",
        )
    )

    figure.add_trace(
        go.Scatter3d(
            x=[points[0].east_m],
            y=[points[0].north_m],
            z=[points[0].up_m],
            mode="markers",
            name="Launch",
            marker={"size": 7, "color": "#57d3a2", "symbol": "diamond"},
            hovertemplate="Launch point<extra></extra>",
        )
    )

    figure.add_trace(
        go.Scatter3d(
            x=[current.east_m],
            y=[current.north_m],
            z=[current.up_m],
            mode="markers",
            name="Current",
            marker={
                "size": 10,
                "color": "#ff4d6d",
                "symbol": "circle",
                "line": {"color": "#ffd0d8", "width": 2},
            },
            hovertemplate="Current position<extra></extra>",
        )
    )

    range_east = list(east)
    range_north = list(north)
    range_up = list(up)
    if show_reference:
        reference_east, reference_north, reference_up = (
            add_representative_trajectory(figure, palette)
        )
        range_east.extend(reference_east)
        range_north.extend(reference_north)
        range_up.extend(reference_up)

    apply_trajectory_layout(figure, palette, range_east, range_north, range_up)
    return figure


def stat_card(
    label: str, value_id: str, unit: str = "", wide: bool = False
) -> html.Div:
    return html.Div(
        [
            html.Div(label, style=STYLES["stat_label"]),
            html.Div(
                [
                    html.Span("--", id=value_id, style=STYLES["stat_value"]),
                    html.Span(unit, style=STYLES["stat_unit"]),
                ],
                style={"display": "flex", "alignItems": "baseline", "gap": "6px"},
            ),
        ],
        style={
            **STYLES["stat_card"],
            **({"gridColumn": "1 / -1"} if wide else {}),
        },
    )


STYLES = {
    "page": {
        "minHeight": "100vh",
        "background": "var(--page-bg)",
        "color": "var(--text)",
        "fontFamily": "Inter, Segoe UI, Arial, sans-serif",
        "padding": "18px",
        "boxSizing": "border-box",
        "transition": "background 160ms ease, color 160ms ease",
    },
    "header": {
        "display": "flex",
        "alignItems": "center",
        "justifyContent": "space-between",
        "gap": "16px",
        "flexWrap": "wrap",
        "marginBottom": "14px",
    },
    "title": {
        "fontSize": "22px",
        "fontWeight": "500",
        "margin": "0",
        "letterSpacing": "0",
    },
    "subtitle": {
        "fontSize": "12px",
        "color": "var(--muted)",
        "marginTop": "4px",
    },
    "header_actions": {
        "display": "flex",
        "alignItems": "center",
        "gap": "10px",
        "flexWrap": "wrap",
    },
    "status": {
        "padding": "7px 10px",
        "border": "1px solid var(--border-strong)",
        "borderRadius": "6px",
        "fontSize": "12px",
        "fontWeight": "500",
        "background": "var(--panel-strong)",
    },
    "button": {
        "border": "1px solid var(--border-strong)",
        "borderRadius": "6px",
        "background": "var(--button-bg)",
        "color": "var(--text)",
        "padding": "8px 12px",
        "cursor": "pointer",
        "fontSize": "12px",
    },
    "content": {
        "display": "grid",
        "gridTemplateColumns": "minmax(0, 1fr) minmax(230px, 290px)",
        "gap": "14px",
        "alignItems": "stretch",
    },
    "graph_panel": {
        "height": "calc(100vh - 106px)",
        "minHeight": "560px",
        "border": "1px solid var(--border)",
        "borderRadius": "6px",
        "overflow": "hidden",
        "background": "var(--graph-bg)",
    },
    "sidebar": {
        "display": "grid",
        "gridTemplateColumns": "1fr 1fr",
        "gap": "10px",
        "alignContent": "start",
    },
    "stat_card": {
        "background": "var(--panel)",
        "border": "1px solid var(--border)",
        "borderRadius": "6px",
        "padding": "13px",
        "minWidth": "0",
    },
    "stat_label": {
        "color": "var(--muted)",
        "fontSize": "11px",
        "marginBottom": "7px",
        "textTransform": "uppercase",
    },
    "stat_value": {
        "fontSize": "19px",
        "fontWeight": "500",
        "whiteSpace": "nowrap",
    },
    "stat_unit": {
        "fontSize": "11px",
        "color": "var(--muted)",
    },
    "wide_card": {
        "gridColumn": "1 / -1",
        "background": "var(--panel)",
        "border": "1px solid var(--border)",
        "borderRadius": "6px",
        "padding": "13px",
    },
}


THEME_VARIABLES = {
    "dark": {
        "--page-bg": "#0a0c0f",
        "--graph-bg": "#0d1014",
        "--panel": "#14191f",
        "--panel-strong": "#151a20",
        "--button-bg": "#20262d",
        "--button-hover": "#2b333c",
        "--reference-accent": "#60a5fa",
        "--text": "#e5e7eb",
        "--text-secondary": "#c4cad2",
        "--muted": "#8c96a3",
        "--border": "#29313a",
        "--border-strong": "#4b5563",
    },
    "light": {
        "--page-bg": "#edf1f5",
        "--graph-bg": "#f6f8fb",
        "--panel": "#ffffff",
        "--panel-strong": "#ffffff",
        "--button-bg": "#ffffff",
        "--button-hover": "#e2e8f0",
        "--reference-accent": "#2563eb",
        "--text": "#172033",
        "--text-secondary": "#475569",
        "--muted": "#64748b",
        "--border": "#d4dce6",
        "--border-strong": "#b8c3d1",
    },
}


def themed_page_style(theme: str) -> dict[str, str]:
    variables = THEME_VARIABLES.get(theme, THEME_VARIABLES["dark"])
    return {**STYLES["page"], **variables}


STATUS_COLORS = {
    "connected": {"color": "#6ee7b7", "borderColor": "#276749"},
    "demo": {"color": "#fbbf24", "borderColor": "#7c5b17"},
    "waiting": {"color": "#93c5fd", "borderColor": "#315d85"},
    "error": {"color": "#fb7185", "borderColor": "#7f2d3a"},
}


def build_dash_app(store: TelemetryStore, source_label: str) -> Dash:
    app = Dash(__name__)
    app.title = f"Rocket Trajectory v{APP_VERSION}"

    app.layout = html.Div(
        [
            html.Div(
                [
                    html.Div(
                        [
                            html.H1(
                                f"Rocket Trajectory v{APP_VERSION}",
                                style=STYLES["title"],
                            ),
                            html.Div(source_label, style=STYLES["subtitle"]),
                        ]
                    ),
                    html.Div(
                        [
                            html.Div("WAITING", id="source-status", style=STYLES["status"]),
                            html.Button(
                                "Light mode",
                                id="theme-toggle",
                                n_clicks=0,
                                title="Switch color theme",
                                style=STYLES["button"],
                            ),
                            html.Button(
                                "Show reference path",
                                id="reference-path-toggle",
                                n_clicks=0,
                                title="Show or hide the representative rocket trajectory",
                                style=STYLES["button"],
                            ),
                            html.Button(
                                "Reset track",
                                id="reset-track",
                                n_clicks=0,
                                style=STYLES["button"],
                            ),
                        ],
                        style=STYLES["header_actions"],
                    ),
                ],
                style=STYLES["header"],
            ),
            html.Div(
                [
                    html.Div(
                        dcc.Graph(
                            id="trajectory-graph",
                            figure=create_trajectory_figure([]),
                            config={
                                "displaylogo": False,
                                "scrollZoom": True,
                                "responsive": True,
                                "modeBarButtonsToRemove": [
                                    "toImage",
                                    "select3d",
                                    "lasso3d",
                                ],
                            },
                            style={"height": "100%", "width": "100%"},
                        ),
                        style=STYLES["graph_panel"],
                    ),
                    html.Div(
                        [
                            stat_card(
                                "Payload latitude", "latitude-value", "deg", wide=True
                            ),
                            stat_card(
                                "Payload longitude", "longitude-value", "deg", wide=True
                            ),
                            stat_card("Payload GNSS altitude", "altitude-value", "m"),
                            stat_card("Relative up", "up-value", "m"),
                            stat_card("Ground speed", "ground-speed-value", "m/s"),
                            stat_card("Vertical speed", "vertical-speed-value", "m/s"),
                            stat_card("Ground range", "range-value", "m"),
                            stat_card("Max relative up", "max-up-value", "m"),
                            html.Div(
                                [
                                    html.Div("PACKETS", style=STYLES["stat_label"]),
                                    html.Div(
                                        [
                                            html.Span(
                                                "0 accepted",
                                                id="accepted-value",
                                                style={"color": "#6ee7b7"},
                                            ),
                                            html.Span(" / ", style={"color": "#56606c"}),
                                            html.Span(
                                                "0 rejected",
                                                id="rejected-value",
                                                style={"color": "#fb7185"},
                                            ),
                                        ],
                                        style={"fontSize": "13px"},
                                    ),
                                ],
                                style=STYLES["wide_card"],
                            ),
                            html.Div(
                                [
                                    html.Div("SOURCE MESSAGE", style=STYLES["stat_label"]),
                                    html.Div(
                                        "Waiting for source",
                                        id="source-message",
                                        style={
                                            "fontSize": "12px",
                                            "color": "var(--text-secondary)",
                                            "overflowWrap": "anywhere",
                                        },
                                    ),
                                ],
                                style=STYLES["wide_card"],
                            ),
                        ],
                        style=STYLES["sidebar"],
                    ),
                ],
                style=STYLES["content"],
            ),
            dcc.Store(id="theme-mode", storage_type="local", data="dark"),
            dcc.Store(id="reference-path-visible", data=False),
            dcc.Interval(id="refresh-timer", interval=250, n_intervals=0),
        ],
        id="app-shell",
        style=themed_page_style("dark"),
    )

    app.index_string = """
<!DOCTYPE html>
<html>
    <head>
        {%metas%}
        <title>{%title%}</title>
        {%favicon%}
        {%css%}
        <style>
            * { box-sizing: border-box; }
            body { margin: 0; background: #0a0c0f; }
            button:hover { background: var(--button-hover) !important; }
            button:focus-visible { outline: 2px solid #57d3a2; outline-offset: 2px; }
            @media (max-width: 900px) {
                #trajectory-graph { min-height: 520px; }
            }
        </style>
    </head>
    <body>
        {%app_entry%}
        <footer>
            {%config%}
            {%scripts%}
            {%renderer%}
        </footer>
        <script>
            const updateLayout = () => {
                const content = document.querySelector('[style*="grid-template-columns"]');
                if (!content) return;
                if (window.innerWidth <= 900) {
                    content.style.gridTemplateColumns = '1fr';
                } else {
                    content.style.gridTemplateColumns = 'minmax(0, 1fr) minmax(230px, 290px)';
                }
            };
            window.addEventListener('resize', updateLayout);
            window.addEventListener('load', updateLayout);
        </script>
    </body>
</html>
"""

    @app.callback(
        Output("theme-mode", "data"),
        Input("theme-toggle", "n_clicks"),
        State("theme-mode", "data"),
        prevent_initial_call=True,
    )
    def toggle_theme(_clicks: int, current_theme: str) -> str:
        return "light" if current_theme == "dark" else "dark"

    @app.callback(
        Output("reference-path-visible", "data"),
        Output("reference-path-toggle", "children"),
        Output("reference-path-toggle", "style"),
        Input("reference-path-toggle", "n_clicks"),
        State("reference-path-visible", "data"),
        prevent_initial_call=True,
    )
    def toggle_reference_path(_clicks: int, currently_visible: bool):
        visible = not bool(currently_visible)
        button_style = {
            **STYLES["button"],
            **(
                {
                    "borderColor": "var(--reference-accent)",
                    "color": "var(--reference-accent)",
                }
                if visible
                else {}
            ),
        }
        label = "Hide reference path" if visible else "Show reference path"
        return visible, label, button_style

    @app.callback(
        Output("app-shell", "style"),
        Output("theme-toggle", "children"),
        Output("trajectory-graph", "figure"),
        Output("source-status", "children"),
        Output("source-status", "style"),
        Output("source-message", "children"),
        Output("latitude-value", "children"),
        Output("longitude-value", "children"),
        Output("altitude-value", "children"),
        Output("up-value", "children"),
        Output("ground-speed-value", "children"),
        Output("vertical-speed-value", "children"),
        Output("range-value", "children"),
        Output("max-up-value", "children"),
        Output("accepted-value", "children"),
        Output("rejected-value", "children"),
        Input("refresh-timer", "n_intervals"),
        Input("reset-track", "n_clicks"),
        Input("theme-mode", "data"),
        Input("reference-path-visible", "data"),
    )
    def update_dashboard(
        _interval: int,
        _reset_clicks: int,
        theme: str,
        show_reference: bool,
    ):
        if ctx.triggered_id == "reset-track":
            store.clear_track()

        theme = theme if theme in THEME_VARIABLES else "dark"
        points, source = store.snapshot()
        status = str(source["status"])
        status_style = {
            **STYLES["status"],
            **STATUS_COLORS.get(status, STATUS_COLORS["waiting"]),
        }

        if not points:
            values = ("--",) * 8
        else:
            current = points[-1]
            ground_speed, vertical_speed = calculate_speed(points)
            ground_range = math.hypot(current.east_m, current.north_m)
            max_up = max(point.up_m for point in points)
            values = (
                f"{current.latitude_deg:.7f}",
                f"{current.longitude_deg:.7f}",
                f"{current.altitude_m:.1f}",
                f"{current.up_m:.1f}",
                f"{ground_speed:.1f}",
                f"{vertical_speed:+.1f}",
                f"{ground_range:.1f}",
                f"{max_up:.1f}",
            )

        return (
            themed_page_style(theme),
            "Dark mode" if theme == "light" else "Light mode",
            create_trajectory_figure(points, theme, bool(show_reference)),
            status.upper(),
            status_style,
            str(source["message"]),
            *values,
            f"{source['accepted']} accepted",
            f"{source['rejected']} rejected",
        )

    return app


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live 3D viewer for 78-byte HYI GNSS telemetry packets."
    )
    parser.add_argument(
        "--serial-port",
        default=DEFAULT_SERIAL_PORT,
        help=f"Serial port (default: {DEFAULT_SERIAL_PORT})",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD_RATE,
        help=f"Serial baud rate (default: {DEFAULT_BAUD_RATE})",
    )
    parser.add_argument(
        "--http-port",
        type=int,
        default=DEFAULT_HTTP_PORT,
        help=f"Dashboard HTTP port (default: {DEFAULT_HTTP_PORT})",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Use a synthetic flight instead of a serial port",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Do not open the dashboard in the default browser",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    load_dashboard_dependencies()
    store = TelemetryStore()

    if args.demo:
        source = DemoReceiver(store)
        source_label = "Demo source / 78-byte HYI packet model"
    else:
        source = SerialReceiver(store, args.serial_port, args.baud)
        source_label = f"Serial source / {args.serial_port} / {args.baud} baud"

    source.start()
    atexit.register(source.stop)

    app = build_dash_app(store, source_label)
    dashboard_url = f"http://127.0.0.1:{args.http_port}"
    print(f"Dashboard: {dashboard_url}")

    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(dashboard_url)).start()

    try:
        app.run(host="127.0.0.1", port=args.http_port, debug=False)
    finally:
        source.stop()


if __name__ == "__main__":
    main()
