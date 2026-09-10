#!/usr/bin/env python3
"""Live 3D GNSS trajectory viewer for the 78-byte HYI telemetry packet."""

from __future__ import annotations

import argparse
import atexit
import base64
from bisect import bisect_right
import math
import re
import struct
import threading
import time
import webbrowser
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional

from telemetry_csv import (
    MAX_REPLAY_SAMPLES,
    ReplaySample,
    parse_telemetry_csv,
    telemetry_points_to_csv,
)
from trajectory_io import (
    MAX_TRAJECTORY_POINTS,
    LoadedTrajectory,
    TrajectoryRecord,
    parse_trajectory_csv,
)

try:
    import serial
    from serial.tools import list_ports as serial_list_ports
except ImportError:
    serial = None
    serial_list_ports = None


def load_dashboard_dependencies() -> None:
    """Load UI packages only when the dashboard is actually started."""
    global Dash, Input, Output, State, ctx, dcc, html, no_update, go
    try:
        # Plotly lazily imports NumPy while constructing figures. Importing it
        # once here avoids a partial-module race when several browser tabs send
        # their first dashboard callbacks at the same time.
        import numpy  # noqa: F401
        from dash import Dash, Input, Output, State, ctx, dcc, html, no_update
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
WGS84_SEMI_MAJOR_M = 6378137.0
WGS84_ECCENTRICITY_SQUARED = 6.69437999014e-3
PAYLOAD_ALTITUDE_OFFSET = 22
PAYLOAD_LATITUDE_OFFSET = 26
PAYLOAD_LONGITUDE_OFFSET = 30

DEFAULT_SERIAL_PORT = "COM9"
DEFAULT_BAUD_RATE = 19200
COMMON_BAUD_RATES = (
    1_200,
    2_400,
    4_800,
    9_600,
    19_200,
    38_400,
    57_600,
    115_200,
    230_400,
    460_800,
    921_600,
)
MIN_SERIAL_BAUD_RATE = 300
MAX_SERIAL_BAUD_RATE = 4_000_000
MAX_SERIAL_PORT_LENGTH = 255
DEFAULT_HTTP_PORT = 8071
APP_VERSION = 3
APP_NAME = "Proist Roket Takımı Bilimsel Görev Yazılımı"
DEFAULT_STALE_TIMEOUT_S = 2.0
DEFAULT_MAX_PLAUSIBLE_SPEED_MPS = 3_000.0
MAX_REASONABLE_ALTITUDE_M = 1_000_000.0
MAX_REPLAY_DURATION_S = 7 * 24 * 60 * 60
MAX_TRAJECTORY_COORDINATE_M = 100_000_000.0
MAX_POINT_CAPACITY = 1_000_000
SEQUENCE_RESYNC_RUN_LENGTH = 3
MAX_UPLOAD_BYTES = 16 * 1024 * 1024
MAX_REQUEST_BYTES = 24 * 1024 * 1024
MAX_RENDER_POINTS = 3_000
WEB_MERCATOR_MAX_LATITUDE_DEG = 85.05112878
# Conservative square viewport that also fits the 320 px mobile layout after
# page padding; larger panels simply receive a little more breathing room.
MAP_FIT_VIEWPORT_PX = 300.0
MAP_TILE_SIZE_PX = 512.0
MAP_FIT_FRACTION = 0.85
MIN_SCENE_AXIS_RATIO = 0.35
MAP_TERRAIN_PITCH_DEG = 62.0
MAP_TERRAIN_BEARING_DEG = -22.0
MAP_REFRESH_INTERVAL_MS = 2_000
MODE_SELECTION_PROMPT_TIMEOUT_S = 6.0
MAP_TERRAIN_TILEJSON_URL = "https://tiles.mapterhorn.com/tilejson.json"
OSM_TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
PARTIAL_REPLAY_MIN_ALTITUDE_DROP_M = 5.0
PARTIAL_REPLAY_NOTICE = (
    "CSV starts during descent; launch/ascent is missing, so the built-in "
    "launch reference is hidden."
)
PARTIAL_REPLAY_PLOT_NOTICE = (
    "Partial replay: launch/ascent missing<br>Built-in reference hidden"
)


def normalize_serial_port(value: object) -> str:
    """Validate a serial device name without assuming a specific OS."""
    if not isinstance(value, str):
        raise ValueError("Select a serial port")
    if any(ord(character) < 32 or ord(character) == 127 for character in value):
        raise ValueError("Serial port name contains control characters")
    serial_port = value.strip()
    if not serial_port:
        raise ValueError("Select a serial port")
    if len(serial_port) > MAX_SERIAL_PORT_LENGTH:
        raise ValueError("Serial port name is too long")
    return serial_port


def normalize_baud_rate(value: object) -> int:
    """Return a practical, integral serial baud rate."""
    if isinstance(value, bool):
        raise ValueError("Baud rate must be a whole number")
    try:
        numeric_value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("Baud rate must be a whole number") from exc
    if not math.isfinite(numeric_value) or not numeric_value.is_integer():
        raise ValueError("Baud rate must be a whole number")
    baud_rate = int(numeric_value)
    if not MIN_SERIAL_BAUD_RATE <= baud_rate <= MAX_SERIAL_BAUD_RATE:
        raise ValueError(
            f"Baud rate must be between {MIN_SERIAL_BAUD_RATE:,} and "
            f"{MAX_SERIAL_BAUD_RATE:,}"
        )
    return baud_rate


def validate_serial_settings(
    serial_port: object,
    baud_rate: object,
) -> tuple[str, int]:
    return normalize_serial_port(serial_port), normalize_baud_rate(baud_rate)


def _serial_port_sort_key(serial_port: str) -> tuple[int, object, str]:
    com_match = re.fullmatch(r"COM(\d+)", serial_port, flags=re.IGNORECASE)
    if com_match:
        return 0, int(com_match.group(1)), serial_port.casefold()
    return 1, serial_port.casefold(), serial_port


def enumerate_serial_port_options(active_port: object) -> list[dict[str, str]]:
    """List detected ports while retaining an explicitly configured device."""
    configured_port = normalize_serial_port(active_port)
    entries: dict[str, tuple[str, str]] = {}
    if serial_list_ports is not None:
        try:
            detected_ports = serial_list_ports.comports()
        except Exception:
            detected_ports = ()
        for port_info in detected_ports:
            try:
                device = normalize_serial_port(getattr(port_info, "device", ""))
            except ValueError:
                continue
            description = str(getattr(port_info, "description", "") or "").strip()
            if description.casefold() in {"", "n/a", device.casefold()}:
                label = device
            else:
                label = f"{device} — {description}"
            entries.setdefault(device.casefold(), (device, label))

    configured_key = configured_port.casefold()
    if configured_key in entries:
        _, label = entries[configured_key]
        entries[configured_key] = (configured_port, label)
    else:
        entries[configured_key] = (
            configured_port,
            f"{configured_port} — configured / not detected",
        )

    ordered_entries = sorted(entries.values(), key=lambda entry: _serial_port_sort_key(entry[0]))
    return [{"label": label, "value": value} for value, label in ordered_entries]


def serial_baud_options(active_baud: object) -> list[dict[str, object]]:
    baud_rate = normalize_baud_rate(active_baud)
    values = sorted(set(COMMON_BAUD_RATES) | {baud_rate})
    return [{"label": f"{value:,} baud", "value": value} for value in values]


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


@dataclass(frozen=True)
class GroundStationLocation:
    """Validated ground-station marker shown independently from launch origin."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: Optional[float] = None
    accuracy_m: Optional[float] = None
    source: str = "manual"


def parse_ground_station_location(
    latitude: object,
    longitude: object,
    altitude: object = None,
    accuracy: object = None,
    source: str = "manual",
) -> GroundStationLocation:
    """Validate browser or manually entered ground-station coordinates."""

    try:
        latitude_deg = float(latitude)
        longitude_deg = float(longitude)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("Enter valid ground-station latitude and longitude") from exc
    if not math.isfinite(latitude_deg) or not -90.0 <= latitude_deg <= 90.0:
        raise ValueError("Ground-station latitude must be between -90 and 90")
    if not math.isfinite(longitude_deg) or not -180.0 <= longitude_deg <= 180.0:
        raise ValueError("Ground-station longitude must be between -180 and 180")

    altitude_m: Optional[float]
    if altitude in (None, ""):
        altitude_m = None
    else:
        try:
            altitude_m = float(altitude)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("Ground-station altitude must be a number") from exc
        if (
            not math.isfinite(altitude_m)
            or not -5_000.0 <= altitude_m <= MAX_REASONABLE_ALTITUDE_M
        ):
            raise ValueError("Ground-station altitude is outside the supported range")

    accuracy_m: Optional[float]
    if accuracy in (None, ""):
        accuracy_m = None
    else:
        try:
            accuracy_m = float(accuracy)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("Ground-station accuracy must be a number") from exc
        if not math.isfinite(accuracy_m) or accuracy_m < 0.0:
            raise ValueError("Ground-station accuracy cannot be negative")

    location_source = str(source or "manual").strip().casefold()
    if location_source not in {"computer", "manual"}:
        raise ValueError("Ground-station source must be computer or manual")
    return GroundStationLocation(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        accuracy_m=accuracy_m,
        source=location_source,
    )


@dataclass(frozen=True)
class _TelemetryStoreCheckpoint:
    """Private snapshot used while replay temporarily owns the shared store."""

    points: tuple[TelemetryPoint, ...]
    reference: Optional[tuple[float, float, float]]
    start_monotonic: Optional[float]
    status: str
    message: str
    accepted_packets: int
    rejected_packets: int
    missing_packets: int
    duplicate_packets: int
    out_of_order_packets: int
    last_packet_counter: Optional[int]
    last_receive_epoch: Optional[float]
    last_source_monotonic: Optional[float]
    launch_point: Optional[TelemetryPoint]
    max_up_m: float
    max_ground_speed_mps: float
    max_vertical_speed_mps: float
    distance_travelled_m: float
    recorded_points: tuple[TelemetryPoint, ...]


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
    if not -5_000.0 <= altitude_m <= MAX_REASONABLE_ALTITUDE_M:
        raise ValueError("GNSS altitude is outside plausible range")
    if abs(latitude_deg) < 1e-9 and abs(longitude_deg) < 1e-9:
        raise ValueError("GNSS fix is unavailable (zero latitude/longitude)")

    return packet_counter, latitude_deg, longitude_deg, altitude_m


class HYIPacketDecoder:
    """Incrementally recovers valid HYI packets from an arbitrary byte stream."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.rejected_frames = 0
        self.stream_length = 0
        self.packet_end_offsets: list[int] = []

    def _discard(self, count: int) -> None:
        if count > 0:
            del self.buffer[:count]

    def feed(self, data: bytes) -> list[bytes]:
        self.packet_end_offsets = []
        if data:
            self.buffer.extend(data)
            self.stream_length += len(data)

        packets: list[bytes] = []
        while True:
            header_index = self.buffer.find(HEADER)
            if header_index < 0:
                keep = min(len(self.buffer), len(HEADER) - 1)
                if keep:
                    self._discard(len(self.buffer) - keep)
                else:
                    self.buffer.clear()
                break

            if header_index:
                self._discard(header_index)

            if len(self.buffer) < PACKET_LENGTH:
                break

            candidate = bytes(self.buffer[:PACKET_LENGTH])
            if (
                candidate[-2:] == FOOTER
                and packet_checksum(candidate) == candidate[75]
            ):
                packets.append(candidate)
                self.packet_end_offsets.append(
                    self.stream_length - len(self.buffer) + PACKET_LENGTH
                )
                self._discard(PACKET_LENGTH)
            else:
                self.rejected_frames += 1
                self._discard(1)

        return packets


def llh_to_ecef(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
) -> tuple[float, float, float]:
    """Convert WGS84 latitude/longitude/ellipsoidal height to ECEF."""
    latitude_rad = math.radians(latitude_deg)
    longitude_rad = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude_rad)
    cos_latitude = math.cos(latitude_rad)
    prime_vertical_radius = WGS84_SEMI_MAJOR_M / math.sqrt(
        1.0 - WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude
    )
    x_m = (prime_vertical_radius + altitude_m) * cos_latitude * math.cos(
        longitude_rad
    )
    y_m = (prime_vertical_radius + altitude_m) * cos_latitude * math.sin(
        longitude_rad
    )
    z_m = (
        prime_vertical_radius * (1.0 - WGS84_ECCENTRICITY_SQUARED)
        + altitude_m
    ) * sin_latitude
    return x_m, y_m, z_m


def llh_to_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    reference_latitude_deg: float,
    reference_longitude_deg: float,
    reference_altitude_m: float,
) -> tuple[float, float, float]:
    """Convert WGS84 latitude/longitude/height to a local ENU frame."""
    reference_latitude_rad = math.radians(reference_latitude_deg)
    reference_longitude_rad = math.radians(reference_longitude_deg)
    x_m, y_m, z_m = llh_to_ecef(latitude_deg, longitude_deg, altitude_m)
    reference_x_m, reference_y_m, reference_z_m = llh_to_ecef(
        reference_latitude_deg,
        reference_longitude_deg,
        reference_altitude_m,
    )
    delta_x = x_m - reference_x_m
    delta_y = y_m - reference_y_m
    delta_z = z_m - reference_z_m

    sin_latitude = math.sin(reference_latitude_rad)
    cos_latitude = math.cos(reference_latitude_rad)
    sin_longitude = math.sin(reference_longitude_rad)
    cos_longitude = math.cos(reference_longitude_rad)

    east_m = -sin_longitude * delta_x + cos_longitude * delta_y
    north_m = (
        -sin_latitude * cos_longitude * delta_x
        - sin_latitude * sin_longitude * delta_y
        + cos_latitude * delta_z
    )
    up_m = (
        cos_latitude * cos_longitude * delta_x
        + cos_latitude * sin_longitude * delta_y
        + sin_latitude * delta_z
    )
    return east_m, north_m, up_m


class TelemetryStore:
    """Thread-safe live track, cumulative flight metrics, and recorder."""

    def __init__(
        self,
        max_points: int = 20_000,
        max_plausible_speed_mps: float = DEFAULT_MAX_PLAUSIBLE_SPEED_MPS,
    ) -> None:
        if not isinstance(max_points, int) or not 1 <= max_points <= MAX_POINT_CAPACITY:
            raise ValueError(
                f"max_points must be between 1 and {MAX_POINT_CAPACITY:,}"
            )
        self._points: deque[TelemetryPoint] = deque(maxlen=max_points)
        self._lock = threading.RLock()
        self._max_plausible_speed_mps = max_plausible_speed_mps
        self._reference: Optional[tuple[float, float, float]] = None
        self._start_monotonic: Optional[float] = None
        self._status = "waiting"
        self._message = "Waiting for source"
        self._accepted_packets = 0
        self._rejected_packets = 0
        self._missing_packets = 0
        self._duplicate_packets = 0
        self._out_of_order_packets = 0
        self._last_packet_counter: Optional[int] = None
        self._resync_candidate: Optional[int] = None
        self._resync_run_start: Optional[int] = None
        self._resync_run_length = 0
        self._resync_duplicate_count = 0
        self._last_receive_epoch: Optional[float] = None
        self._last_source_monotonic: Optional[float] = None
        self._launch_point: Optional[TelemetryPoint] = None
        self._max_up_m = 0.0
        self._max_ground_speed_mps = 0.0
        self._max_vertical_speed_mps = 0.0
        self._distance_travelled_m = 0.0
        self._recording = False
        self._recording_start_monotonic: Optional[float] = None
        self._recorded_points: deque[TelemetryPoint] = deque(maxlen=max_points)

    def set_status(self, status: str, message: str) -> None:
        with self._lock:
            self._status = status
            self._message = message

    def add_rejected(self, count: int = 1) -> None:
        with self._lock:
            self._rejected_packets += count

    def _track_sequence_locked(self, normalized_counter: int) -> bool:
        """Advance the packet counter baseline; the caller must hold ``_lock``."""
        if self._last_packet_counter is not None:
            counter_delta = (
                normalized_counter - self._last_packet_counter
            ) & 0xFF
            candidate_continues = (
                self._resync_candidate is not None
                and (normalized_counter - self._resync_candidate) & 0xFF == 1
            )
            # A candidate may cross the old baseline exactly once (delta 0),
            # but a normally-forward counter wins over the heuristic.  This
            # avoids treating one stale 255 plus the valid 0,1 stream as a
            # 254-packet loss while still allowing 254,255,0 to resynchronize.
            if candidate_continues and (counter_delta == 0 or counter_delta > 128):
                if counter_delta == 0:
                    self._duplicate_packets += 1
                    self._resync_duplicate_count += 1
                else:
                    self._out_of_order_packets += 1
                self._resync_candidate = normalized_counter
                self._resync_run_length += 1
                if self._resync_run_length >= SEQUENCE_RESYNC_RUN_LENGTH:
                    run_start = self._resync_run_start
                    if run_start is not None:
                        self._missing_packets += (
                            (run_start - self._last_packet_counter) & 0xFF
                        ) - 1
                    self._duplicate_packets -= self._resync_duplicate_count
                    self._out_of_order_packets -= (
                        self._resync_run_length - self._resync_duplicate_count
                    )
                    self._last_packet_counter = normalized_counter
                    self._clear_resync_candidate_locked()
                    return True
                return False
            if counter_delta == 0:
                self._clear_resync_candidate_locked()
                self._duplicate_packets += 1
                return False
            if counter_delta > 128:
                self._out_of_order_packets += 1
                self._resync_run_start = normalized_counter
                self._resync_run_length = 1
                self._resync_duplicate_count = 0
                self._resync_candidate = normalized_counter
                return False
            if counter_delta > 1:
                self._missing_packets += counter_delta - 1
        self._clear_resync_candidate_locked()
        self._last_packet_counter = normalized_counter
        return True

    def _clear_resync_candidate_locked(self) -> None:
        self._resync_candidate = None
        self._resync_run_start = None
        self._resync_run_length = 0
        self._resync_duplicate_count = 0

    def add_rejected_packet(self, packet_counter: int) -> None:
        """Count a validated wire frame whose payload is unusable."""
        normalized_counter = int(packet_counter) & 0xFF
        with self._lock:
            self._rejected_packets += 1
            self._track_sequence_locked(normalized_counter)

    def append(
        self,
        packet_counter: int,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
        *,
        monotonic_s: Optional[float] = None,
        epoch_s: Optional[float] = None,
        elapsed_s: Optional[float] = None,
        validate_sequence: bool = True,
        validate_motion: bool = True,
    ) -> bool:
        """Append one fix and return whether it passed sequence/motion validation."""
        source_monotonic = time.monotonic() if monotonic_s is None else monotonic_s
        point_epoch = time.time() if epoch_s is None else epoch_s
        receive_epoch = time.time()

        with self._lock:
            normalized_counter = int(packet_counter) & 0xFF
            if validate_sequence:
                if not self._track_sequence_locked(normalized_counter):
                    self._rejected_packets += 1
                    return False
            else:
                self._last_packet_counter = normalized_counter

            coordinates = (latitude_deg, longitude_deg, altitude_m)
            if (
                not all(math.isfinite(float(value)) for value in coordinates)
                or not -90.0 <= latitude_deg <= 90.0
                or not -180.0 <= longitude_deg <= 180.0
                or not -5_000.0 <= altitude_m <= MAX_REASONABLE_ALTITUDE_M
                or (
                    abs(latitude_deg) < 1e-9
                    and abs(longitude_deg) < 1e-9
                )
            ):
                self._rejected_packets += 1
                return False

            if self._reference is None:
                self._reference = (latitude_deg, longitude_deg, altitude_m)
                self._start_monotonic = source_monotonic

            reference_lat, reference_lon, reference_alt = self._reference
            east_m, north_m, up_m = llh_to_enu(
                latitude_deg,
                longitude_deg,
                altitude_m,
                reference_lat,
                reference_lon,
                reference_alt,
            )
            if elapsed_s is None:
                start_monotonic = (
                    source_monotonic
                    if self._start_monotonic is None
                    else self._start_monotonic
                )
                point_time_s = max(0.0, source_monotonic - start_monotonic)
            else:
                point_time_s = max(0.0, float(elapsed_s))

            previous = self._points[-1] if self._points else None
            if previous is not None and validate_motion:
                delta_time = point_time_s - previous.time_s
                if delta_time <= 0.0:
                    self._rejected_packets += 1
                    return False
                distance_3d = math.sqrt(
                    (east_m - previous.east_m) ** 2
                    + (north_m - previous.north_m) ** 2
                    + (up_m - previous.up_m) ** 2
                )
                if (
                    self._max_plausible_speed_mps > 0.0
                    and distance_3d / delta_time
                    > self._max_plausible_speed_mps
                ):
                    self._rejected_packets += 1
                    return False

            point = TelemetryPoint(
                time_s=point_time_s,
                epoch_s=point_epoch,
                packet_counter=normalized_counter,
                latitude_deg=latitude_deg,
                longitude_deg=longitude_deg,
                altitude_m=altitude_m,
                east_m=east_m,
                north_m=north_m,
                up_m=up_m,
            )

            if previous is not None:
                delta_time = point.time_s - previous.time_s
                horizontal_distance = math.hypot(
                    point.east_m - previous.east_m,
                    point.north_m - previous.north_m,
                )
                self._distance_travelled_m += math.sqrt(
                    horizontal_distance**2 + (point.up_m - previous.up_m) ** 2
                )
                if delta_time > 0.0:
                    self._max_ground_speed_mps = max(
                        self._max_ground_speed_mps,
                        horizontal_distance / delta_time,
                    )
                    self._max_vertical_speed_mps = max(
                        self._max_vertical_speed_mps,
                        abs(point.up_m - previous.up_m) / delta_time,
                    )

            self._points.append(point)
            if self._launch_point is None:
                self._launch_point = point
            self._max_up_m = max(self._max_up_m, point.up_m)
            self._accepted_packets += 1
            self._last_receive_epoch = receive_epoch
            self._last_source_monotonic = source_monotonic
            if self._recording:
                recording_start = self._recording_start_monotonic
                if recording_start is None:
                    recording_start = source_monotonic
                    self._recording_start_monotonic = recording_start
                if source_monotonic >= recording_start:
                    self._recorded_points.append(
                        TelemetryPoint(
                            time_s=source_monotonic - recording_start,
                            epoch_s=point.epoch_s,
                            packet_counter=point.packet_counter,
                            latitude_deg=point.latitude_deg,
                            longitude_deg=point.longitude_deg,
                            altitude_m=point.altitude_m,
                            east_m=point.east_m,
                            north_m=point.north_m,
                            up_m=point.up_m,
                        )
                    )
            if self._message.startswith("Track cleared"):
                self._message = "Receiving telemetry"
            return True

    def clear_track(self, *, reset_counters: bool = True) -> None:
        with self._lock:
            self._points.clear()
            self._reference = None
            self._start_monotonic = None
            self._last_packet_counter = None
            self._clear_resync_candidate_locked()
            self._last_receive_epoch = None
            self._last_source_monotonic = None
            self._launch_point = None
            self._max_up_m = 0.0
            self._max_ground_speed_mps = 0.0
            self._max_vertical_speed_mps = 0.0
            self._distance_travelled_m = 0.0
            if reset_counters:
                self._accepted_packets = 0
                self._rejected_packets = 0
                self._missing_packets = 0
                self._duplicate_packets = 0
                self._out_of_order_packets = 0
            self._message = "Track cleared; waiting for next fix"

    def start_recording(self, *, include_existing: bool = False) -> int:
        with self._lock:
            if include_existing and self._points:
                first_time_s = self._points[0].time_s
                self._recorded_points = deque(
                    (
                        TelemetryPoint(
                            time_s=max(0.0, point.time_s - first_time_s),
                            epoch_s=point.epoch_s,
                            packet_counter=point.packet_counter,
                            latitude_deg=point.latitude_deg,
                            longitude_deg=point.longitude_deg,
                            altitude_m=point.altitude_m,
                            east_m=point.east_m,
                            north_m=point.north_m,
                            up_m=point.up_m,
                        )
                        for point in self._points
                    ),
                    maxlen=self._points.maxlen,
                )
                recorded_duration_s = self._recorded_points[-1].time_s
                if self._last_source_monotonic is not None:
                    self._recording_start_monotonic = (
                        self._last_source_monotonic - recorded_duration_s
                    )
                else:
                    self._recording_start_monotonic = (
                        time.monotonic() - recorded_duration_s
                    )
            else:
                self._recorded_points = deque(maxlen=self._points.maxlen)
                self._recording_start_monotonic = time.monotonic()
            self._recording = True
            return len(self._recorded_points)

    def stop_recording(self) -> int:
        with self._lock:
            self._recording = False
            self._recording_start_monotonic = None
            return len(self._recorded_points)

    def recording_snapshot(self) -> list[TelemetryPoint]:
        with self._lock:
            return list(self._recorded_points)

    def checkpoint(self) -> _TelemetryStoreCheckpoint:
        """Capture live mission state before replay borrows this store."""
        with self._lock:
            return _TelemetryStoreCheckpoint(
                points=tuple(self._points),
                reference=self._reference,
                start_monotonic=self._start_monotonic,
                status=self._status,
                message=self._message,
                accepted_packets=self._accepted_packets,
                rejected_packets=self._rejected_packets,
                missing_packets=self._missing_packets,
                duplicate_packets=self._duplicate_packets,
                out_of_order_packets=self._out_of_order_packets,
                last_packet_counter=self._last_packet_counter,
                last_receive_epoch=self._last_receive_epoch,
                last_source_monotonic=self._last_source_monotonic,
                launch_point=self._launch_point,
                max_up_m=self._max_up_m,
                max_ground_speed_mps=self._max_ground_speed_mps,
                max_vertical_speed_mps=self._max_vertical_speed_mps,
                distance_travelled_m=self._distance_travelled_m,
                recorded_points=tuple(self._recorded_points),
            )

    def restore_checkpoint(self, checkpoint: _TelemetryStoreCheckpoint) -> None:
        """Restore a live mission without resuming an interrupted recording."""
        if not isinstance(checkpoint, _TelemetryStoreCheckpoint):
            raise TypeError("Invalid telemetry checkpoint")
        with self._lock:
            self._points = deque(checkpoint.points, maxlen=self._points.maxlen)
            self._reference = checkpoint.reference
            self._start_monotonic = checkpoint.start_monotonic
            self._status = checkpoint.status
            self._message = checkpoint.message
            self._accepted_packets = checkpoint.accepted_packets
            self._rejected_packets = checkpoint.rejected_packets
            self._missing_packets = checkpoint.missing_packets
            self._duplicate_packets = checkpoint.duplicate_packets
            self._out_of_order_packets = checkpoint.out_of_order_packets
            self._last_packet_counter = checkpoint.last_packet_counter
            self._last_receive_epoch = checkpoint.last_receive_epoch
            self._last_source_monotonic = checkpoint.last_source_monotonic
            self._launch_point = checkpoint.launch_point
            self._max_up_m = checkpoint.max_up_m
            self._max_ground_speed_mps = checkpoint.max_ground_speed_mps
            self._max_vertical_speed_mps = checkpoint.max_vertical_speed_mps
            self._distance_travelled_m = checkpoint.distance_travelled_m
            self._recording = False
            self._recording_start_monotonic = None
            self._recorded_points = deque(
                checkpoint.recorded_points,
                maxlen=self._points.maxlen,
            )
            self._clear_resync_candidate_locked()

    def reset_sequence_tracking(self) -> None:
        """Accept the next packet as a new counter baseline after reconnect."""
        with self._lock:
            self._last_packet_counter = None
            self._clear_resync_candidate_locked()

    def latest_source_monotonic(self) -> Optional[float]:
        with self._lock:
            return self._last_source_monotonic

    def snapshot(self) -> tuple[list[TelemetryPoint], dict[str, object]]:
        with self._lock:
            return list(self._points), {
                "status": self._status,
                "message": self._message,
                "accepted": self._accepted_packets,
                "rejected": self._rejected_packets,
                "missing": self._missing_packets,
                "duplicates": self._duplicate_packets,
                "out_of_order": self._out_of_order_packets,
                "last_receive_epoch": self._last_receive_epoch,
                "launch_point": self._launch_point,
                "max_up_m": self._max_up_m,
                "max_ground_speed_mps": self._max_ground_speed_mps,
                "max_vertical_speed_mps": self._max_vertical_speed_mps,
                "distance_travelled_m": self._distance_travelled_m,
                "recording": self._recording,
                "recorded_count": len(self._recorded_points),
            }


class LiveDvrController:
    """Read-only time-shift cursor over the bounded live telemetry history."""

    def __init__(self, time_fn: Callable[[], float] = time.monotonic) -> None:
        self._time_fn = time_fn
        self._lock = threading.RLock()
        self._at_live_edge = True
        self._playing = True
        self._playhead_s = 0.0
        self._speed = 1.0
        self._anchor_monotonic = float(self._time_fn())
        self._anchor_playhead_s = 0.0
        self._last_live_edge_s: Optional[float] = None

    @staticmethod
    def _bounds(points: list[TelemetryPoint]) -> tuple[float, float]:
        if not points:
            return 0.0, 0.0
        return float(points[0].time_s), float(points[-1].time_s)

    def _reset_locked(self, live_edge_s: float = 0.0) -> None:
        now = float(self._time_fn())
        self._at_live_edge = True
        self._playing = True
        self._playhead_s = max(0.0, float(live_edge_s))
        self._anchor_monotonic = now
        self._anchor_playhead_s = self._playhead_s
        self._last_live_edge_s = live_edge_s if live_edge_s > 0.0 else None

    def reset(self) -> None:
        with self._lock:
            self._reset_locked()

    def _sync_bounds_locked(
        self,
        points: list[TelemetryPoint],
    ) -> tuple[float, float]:
        minimum_s, live_edge_s = self._bounds(points)
        if not points:
            self._reset_locked()
            return minimum_s, live_edge_s
        if (
            self._last_live_edge_s is not None
            and live_edge_s + 1e-6 < self._last_live_edge_s
        ):
            # A new source/session restarted its elapsed-time clock.
            self._reset_locked(live_edge_s)
            return minimum_s, live_edge_s
        self._last_live_edge_s = live_edge_s
        if self._at_live_edge:
            self._playhead_s = live_edge_s
            self._anchor_playhead_s = live_edge_s
            self._anchor_monotonic = float(self._time_fn())
        else:
            clamped = min(max(self._playhead_s, minimum_s), live_edge_s)
            if not math.isclose(clamped, self._playhead_s, abs_tol=1e-9):
                self._playhead_s = clamped
                self._anchor_playhead_s = clamped
                self._anchor_monotonic = float(self._time_fn())
            if self._playing and math.isclose(
                self._playhead_s,
                live_edge_s,
                abs_tol=1e-6,
            ):
                self._at_live_edge = True
                self._playing = True
        return minimum_s, live_edge_s

    def seek(
        self,
        target_s: object,
        points: list[TelemetryPoint],
    ) -> float:
        try:
            requested_s = float(target_s)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("Timeline position must be a finite number") from exc
        if not math.isfinite(requested_s):
            raise ValueError("Timeline position must be a finite number")
        with self._lock:
            minimum_s, live_edge_s = self._sync_bounds_locked(points)
            if not points:
                return 0.0
            self._playhead_s = min(max(requested_s, minimum_s), live_edge_s)
            self._at_live_edge = math.isclose(
                self._playhead_s,
                live_edge_s,
                abs_tol=1e-6,
            )
            if self._at_live_edge:
                self._playing = True
            self._anchor_playhead_s = self._playhead_s
            self._anchor_monotonic = float(self._time_fn())
            return self._playhead_s

    def toggle(self, points: list[TelemetryPoint]) -> bool:
        with self._lock:
            _, live_edge_s = self._sync_bounds_locked(points)
            if not points:
                self._playing = False
                return False
            if self._at_live_edge:
                # Pause the view at this instant while ingestion keeps running.
                self._at_live_edge = False
                self._playing = False
                self._playhead_s = live_edge_s
            else:
                self._playing = not self._playing
            self._anchor_playhead_s = self._playhead_s
            self._anchor_monotonic = float(self._time_fn())
            return self._playing

    def tick(self, points: list[TelemetryPoint]) -> None:
        with self._lock:
            minimum_s, live_edge_s = self._sync_bounds_locked(points)
            if not points or self._at_live_edge or not self._playing:
                return
            now = float(self._time_fn())
            elapsed_s = max(0.0, now - self._anchor_monotonic)
            advanced_s = self._anchor_playhead_s + elapsed_s * self._speed
            if advanced_s >= live_edge_s - 1e-6:
                self._at_live_edge = True
                self._playing = True
                self._playhead_s = live_edge_s
                self._anchor_playhead_s = live_edge_s
                self._anchor_monotonic = now
                return
            self._playhead_s = min(max(advanced_s, minimum_s), live_edge_s)

    def go_live(self, points: list[TelemetryPoint]) -> float:
        with self._lock:
            _, live_edge_s = self._bounds(points)
            self._reset_locked(live_edge_s)
            return self._playhead_s

    def set_speed(self, speed: object) -> float:
        try:
            normalized_speed = float(speed)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("Playback speed must be a positive number") from exc
        if not math.isfinite(normalized_speed) or normalized_speed <= 0.0:
            raise ValueError("Playback speed must be a positive number")
        with self._lock:
            now = float(self._time_fn())
            if not self._at_live_edge and self._playing:
                elapsed_s = max(0.0, now - self._anchor_monotonic)
                self._playhead_s = (
                    self._anchor_playhead_s + elapsed_s * self._speed
                )
            self._speed = normalized_speed
            self._anchor_playhead_s = self._playhead_s
            self._anchor_monotonic = now
            return self._speed

    def status(self, points: list[TelemetryPoint]) -> dict[str, object]:
        with self._lock:
            minimum_s, live_edge_s = self._sync_bounds_locked(points)
            return {
                "at_live_edge": self._at_live_edge,
                "playing": self._playing,
                "playhead_s": self._playhead_s,
                "speed": self._speed,
                "minimum_s": minimum_s,
                "live_edge_s": live_edge_s,
                "behind_s": max(0.0, live_edge_s - self._playhead_s),
                "count": len(points),
            }

    def visible_points(
        self,
        points: list[TelemetryPoint],
    ) -> list[TelemetryPoint]:
        with self._lock:
            self._sync_bounds_locked(points)
            if self._at_live_edge or not points:
                return list(points)
            timestamps = [float(point.time_s) for point in points]
            end_index = bisect_right(timestamps, self._playhead_s + 1e-9)
            return list(points[:end_index])


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
                self.store.reset_sequence_tracking()
                self.store.set_status(
                    "waiting",
                    f"{self.serial_port} open; waiting for first valid fix",
                )
                has_valid_fix = False
                last_packet_monotonic = self.store.latest_source_monotonic()
                last_packet_end_offset: Optional[int] = None

                while not self.stop_event.is_set():
                    waiting = self.serial_connection.in_waiting
                    chunk = self.serial_connection.read(max(1, min(waiting, 4096)))
                    if not chunk:
                        continue

                    decoded_packets = decoder.feed(chunk)
                    decoded_at_monotonic = time.monotonic()
                    decoded_at_epoch = time.time()
                    byte_wire_time_s = 10.0 / max(1, self.baud_rate)
                    unread_serial_bytes = self.serial_connection.in_waiting
                    for packet, packet_end_offset in zip(
                        decoded_packets,
                        decoder.packet_end_offsets,
                    ):
                        bytes_after_packet = (
                            decoder.stream_length - packet_end_offset
                            + unread_serial_bytes
                        )
                        packet_monotonic = decoded_at_monotonic - (
                            bytes_after_packet * byte_wire_time_s
                        )
                        if last_packet_monotonic is not None:
                            if last_packet_end_offset is None:
                                minimum_spacing_s = PACKET_LENGTH * byte_wire_time_s
                            else:
                                minimum_spacing_s = (
                                    packet_end_offset - last_packet_end_offset
                                ) * byte_wire_time_s
                            packet_monotonic = max(
                                packet_monotonic,
                                last_packet_monotonic + minimum_spacing_s,
                            )
                        last_packet_monotonic = packet_monotonic
                        last_packet_end_offset = packet_end_offset
                        packet_epoch = decoded_at_epoch + (
                            packet_monotonic - decoded_at_monotonic
                        )
                        try:
                            counter, lat, lon, alt = parse_hyi_packet(packet)
                            accepted = self.store.append(
                                counter,
                                lat,
                                lon,
                                alt,
                                monotonic_s=packet_monotonic,
                                epoch_s=packet_epoch,
                            )
                            if accepted and not has_valid_fix:
                                has_valid_fix = True
                                self.store.set_status(
                                    "connected",
                                    f"{self.serial_port} receiving at "
                                    f"{self.baud_rate} baud",
                                )
                        except ValueError:
                            self.store.add_rejected_packet(packet[5])

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
    """Generates a repeatable flight path without serial hardware.

    ``mission_started_monotonic`` deliberately lives outside an individual
    receiver lifetime when supplied by the application. Replay temporarily
    stops the receiver, but returning from replay must not launch a brand-new
    synthetic rocket from t=0.
    """

    def __init__(
        self,
        store: TelemetryStore,
        update_hz: float = 8.0,
        mission_started_monotonic: Optional[float] = None,
    ) -> None:
        super().__init__(name="demo-receiver", daemon=True)
        if not math.isfinite(update_hz) or update_hz <= 0.0:
            raise ValueError("Demo update rate must be finite and positive")
        if mission_started_monotonic is None:
            mission_started_monotonic = time.monotonic()
        if not math.isfinite(mission_started_monotonic):
            raise ValueError("Demo mission start must be finite")
        self.store = store
        self.update_hz = update_hz
        self.mission_started_monotonic = float(mission_started_monotonic)
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
        return max(0.0, 1_443.75 - 12.5 * dt)

    def sample_at(self, monotonic_s: float) -> tuple[float, float, float]:
        """Return the synthetic LLH fix at one shared mission-clock instant."""
        base_lat = 40.9621053
        base_lon = 29.1294180
        base_alt = 1_400.0
        reference_lat_rad = math.radians(base_lat)
        landing_time_s = 25.0 + 1_443.75 / 12.5
        flight_time = max(0.0, float(monotonic_s) - self.mission_started_monotonic)
        motion_time_s = min(flight_time, landing_time_s)
        east_m = 3.8 * motion_time_s + 11.0 * math.sin(motion_time_s / 8.0)
        north_m = 2.2 * motion_time_s + 7.0 * math.sin(motion_time_s / 11.0)
        relative_altitude_m = self.altitude_profile(flight_time)

        latitude_deg = base_lat + math.degrees(north_m / EARTH_RADIUS_M)
        longitude_deg = base_lon + math.degrees(
            east_m / (EARTH_RADIUS_M * math.cos(reference_lat_rad))
        )
        return latitude_deg, longitude_deg, base_alt + relative_altitude_m

    def run(self) -> None:
        counter = 0
        self.store.set_status("demo", "Synthetic HYI flight · DEMO DATA")

        while not self.stop_event.is_set():
            latitude_deg, longitude_deg, altitude_m = self.sample_at(time.monotonic())

            self.store.append(
                counter & 0xFF,
                latitude_deg,
                longitude_deg,
                altitude_m,
            )
            counter += 1
            self.stop_event.wait(1.0 / self.update_hz)


def create_demo_source_factory(
    store: TelemetryStore,
    update_hz: float = 8.0,
    mission_started_monotonic: Optional[float] = None,
) -> Callable[[], DemoReceiver]:
    """Build receivers that all continue one synthetic mission timeline."""
    shared_mission_start = (
        time.monotonic()
        if mission_started_monotonic is None
        else float(mission_started_monotonic)
    )

    def create_receiver() -> DemoReceiver:
        return DemoReceiver(
            store,
            update_hz=update_hz,
            mission_started_monotonic=shared_mission_start,
        )

    return create_receiver


@dataclass(frozen=True)
class TrajectoryOverlayPoint:
    time_s: Optional[float]
    east_m: float
    north_m: float
    up_m: float
    latitude_deg: Optional[float] = None
    longitude_deg: Optional[float] = None
    altitude_m: Optional[float] = None


class TrajectoryStore:
    """Holds an optional user-supplied OpenRocket or generic reference path."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._name = "Built-in reference"
        self._points: list[TrajectoryOverlayPoint] = []

    @staticmethod
    def _validate_local_coordinates(
        east_m: float,
        north_m: float,
        up_m: float,
    ) -> tuple[float, float, float]:
        values = (east_m, north_m, up_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Trajectory conversion produced non-finite coordinates")
        if any(abs(value) > MAX_TRAJECTORY_COORDINATE_M for value in values):
            raise ValueError(
                "Trajectory exceeds the supported 100,000 km local range"
            )
        return values

    @staticmethod
    def _validate_time(time_s: Optional[float]) -> Optional[float]:
        if time_s is None:
            return None
        value = float(time_s)
        if not math.isfinite(value):
            raise ValueError("Trajectory time contains a non-finite value")
        if abs(value) > MAX_REPLAY_DURATION_S:
            raise ValueError("Trajectory time exceeds the supported 7-day range")
        return value

    def load(self, trajectory: LoadedTrajectory) -> int:
        if len(trajectory.records) > MAX_TRAJECTORY_POINTS:
            raise ValueError(
                f"Trajectory exceeds the {MAX_TRAJECTORY_POINTS:,}-point limit"
            )
        records = list(trajectory.records)
        if not records:
            raise ValueError("Trajectory contains no points")

        converted: list[TrajectoryOverlayPoint] = []
        if trajectory.mode == "llh":
            first = records[0]
            if (
                first.latitude_deg is None
                or first.longitude_deg is None
                or first.altitude_m is None
            ):
                raise ValueError("Trajectory LLH origin is incomplete")
            for record in records:
                if (
                    record.latitude_deg is None
                    or record.longitude_deg is None
                    or record.altitude_m is None
                ):
                    continue
                east_m, north_m, up_m = llh_to_enu(
                    record.latitude_deg,
                    record.longitude_deg,
                    record.altitude_m,
                    first.latitude_deg,
                    first.longitude_deg,
                    first.altitude_m,
                )
                east_m, north_m, up_m = self._validate_local_coordinates(
                    east_m,
                    north_m,
                    up_m,
                )
                converted.append(
                    TrajectoryOverlayPoint(
                        self._validate_time(record.time_s),
                        east_m,
                        north_m,
                        up_m,
                        record.latitude_deg,
                        record.longitude_deg,
                        record.altitude_m,
                    )
                )
        else:
            first_east = records[0].east_m or 0.0
            first_north = records[0].north_m or 0.0
            first_up = records[0].up_m or 0.0
            for record in records:
                if (
                    record.east_m is None
                    or record.north_m is None
                    or record.up_m is None
                ):
                    continue
                east_m, north_m, up_m = self._validate_local_coordinates(
                    record.east_m - first_east,
                    record.north_m - first_north,
                    record.up_m - first_up,
                )
                converted.append(
                    TrajectoryOverlayPoint(
                        self._validate_time(record.time_s),
                        east_m,
                        north_m,
                        up_m,
                    )
                )

        if len(converted) < 2:
            raise ValueError("Trajectory must contain at least two usable points")
        with self._lock:
            self._name = trajectory.name
            self._points = converted
        return len(converted)

    def clear(self) -> None:
        with self._lock:
            self._name = "Built-in reference"
            self._points = []

    def snapshot(self) -> tuple[list[TrajectoryOverlayPoint], str]:
        with self._lock:
            return list(self._points), self._name


class ReplayController:
    """Server-side, seekable replay clock advanced by the Dash timer."""

    def __init__(self, store: TelemetryStore) -> None:
        self.store = store
        self._lock = threading.RLock()
        self._samples: tuple[ReplaySample, ...] = ()
        self._name = "No replay loaded"
        self._index = 0
        self._playhead_s = 0.0
        self._duration_s = 0.0
        self._speed = 1.0
        self._playing = False
        self._anchor_monotonic = time.monotonic()
        self._anchor_playhead_s = 0.0
        self._launch_known = True

    @staticmethod
    def _detect_launch_origin(samples: tuple[ReplaySample, ...]) -> bool:
        """Return false for a conservatively detected descent-only segment.

        A legacy CSV with no launch metadata cannot be re-anchored truthfully.
        We therefore only classify a file as partial when it contains enough
        samples, begins near its maximum altitude, and is overwhelmingly
        descending by a meaningful amount.
        """
        if len(samples) < 5:
            return True
        altitude = [sample.altitude_m for sample in samples]
        minimum = min(altitude)
        maximum = max(altitude)
        span = maximum - minimum
        if span < PARTIAL_REPLAY_MIN_ALTITUDE_DROP_M:
            return True
        peak_index = max(range(len(altitude)), key=altitude.__getitem__)
        early_peak_limit = max(1, int((len(altitude) - 1) * 0.1))
        descending_steps = sum(
            1
            for previous, current in zip(altitude, altitude[1:])
            if current < previous
        )
        descending_ratio = descending_steps / (len(altitude) - 1)
        starts_well_above_minimum = altitude[0] - minimum >= max(
            PARTIAL_REPLAY_MIN_ALTITUDE_DROP_M,
            span * 0.25,
        )
        ends_significantly_lower = (
            altitude[0] - altitude[-1] >= PARTIAL_REPLAY_MIN_ALTITUDE_DROP_M
        )
        is_partial_descent = (
            peak_index <= early_peak_limit
            and descending_ratio >= 0.7
            and starts_well_above_minimum
            and ends_significantly_lower
        )
        return not is_partial_descent

    @staticmethod
    def prepare_samples(samples: list[ReplaySample]) -> tuple[ReplaySample, ...]:
        """Validate and normalize replay time without changing the active source."""
        if not samples:
            raise ValueError("Replay contains no samples")
        if len(samples) > MAX_REPLAY_SAMPLES:
            raise ValueError(
                f"Replay exceeds the {MAX_REPLAY_SAMPLES:,}-sample limit"
            )
        ordered = sorted(samples, key=lambda sample: sample.time_s)
        start_time_s = ordered[0].time_s
        normalized_samples: list[ReplaySample] = []
        for sample in ordered:
            normalized_time_s = sample.time_s - start_time_s
            if (
                not math.isfinite(normalized_time_s)
                or normalized_time_s < 0.0
                or normalized_time_s > MAX_REPLAY_DURATION_S
            ):
                raise ValueError(
                    "Replay time span must be finite and no longer than 7 days"
                )
            normalized_samples.append(
                ReplaySample(
                    time_s=normalized_time_s,
                    packet_counter=sample.packet_counter,
                    latitude_deg=sample.latitude_deg,
                    longitude_deg=sample.longitude_deg,
                    altitude_m=sample.altitude_m,
                    epoch_s=sample.epoch_s,
                )
            )
        return tuple(normalized_samples)

    def load_prepared(
        self,
        normalized: tuple[ReplaySample, ...],
        name: str,
    ) -> None:
        if not normalized:
            raise ValueError("Replay contains no samples")
        with self._lock:
            self._samples = normalized
            self._name = name
            self._index = 0
            self._playhead_s = 0.0
            self._duration_s = normalized[-1].time_s
            self._playing = False
            self._launch_known = self._detect_launch_origin(normalized)
            self._reset_anchor()
            self._rebuild_store(0.0)
            message = (
                f"Replay ready: {name}"
                if self._launch_known
                else f"Partial replay ready: {name} · {PARTIAL_REPLAY_NOTICE}"
            )
            self.store.set_status("replay", message)

    def load(self, samples: list[ReplaySample], name: str) -> None:
        self.load_prepared(self.prepare_samples(samples), name)

    def unload(self) -> None:
        with self._lock:
            self._samples = ()
            self._name = "No replay loaded"
            self._index = 0
            self._playhead_s = 0.0
            self._duration_s = 0.0
            self._playing = False
            self._launch_known = True

    def _reset_anchor(self) -> None:
        self._anchor_monotonic = time.monotonic()
        self._anchor_playhead_s = self._playhead_s

    def _append_sample(self, sample: ReplaySample) -> None:
        self.store.append(
            sample.packet_counter,
            sample.latitude_deg,
            sample.longitude_deg,
            sample.altitude_m,
            epoch_s=sample.epoch_s,
            elapsed_s=sample.time_s,
            validate_sequence=False,
            validate_motion=False,
        )

    def _emit_until(self, target_s: float) -> None:
        while (
            self._index < len(self._samples)
            and self._samples[self._index].time_s <= target_s + 1e-9
        ):
            self._append_sample(self._samples[self._index])
            self._index += 1

    def _rebuild_store(self, target_s: float) -> None:
        self.store.clear_track(reset_counters=True)
        self._index = 0
        self._emit_until(target_s)

    def toggle(self) -> bool:
        with self._lock:
            if not self._samples:
                return False
            self.tick()
            if self._playhead_s >= self._duration_s and not self._playing:
                self.seek(0.0)
            self._playing = not self._playing
            self._reset_anchor()
            state = "Playing" if self._playing else "Paused"
            self.store.set_status("replay", f"{state}: {self._name}")
            return self._playing

    def pause(self) -> None:
        with self._lock:
            self.tick()
            self._playing = False
            self._reset_anchor()

    def seek(self, playhead_s: float) -> None:
        with self._lock:
            if not self._samples:
                return
            requested_s = float(playhead_s)
            if not math.isfinite(requested_s):
                raise ValueError("Replay position must be finite")
            target_s = min(max(0.0, requested_s), self._duration_s)
            self._playhead_s = target_s
            self._rebuild_store(target_s)
            self._reset_anchor()
            self.store.set_status("replay", f"Replay position: {target_s:.1f} s")

    def set_speed(self, speed: float) -> None:
        with self._lock:
            self.tick()
            requested_speed = float(speed)
            if not math.isfinite(requested_speed):
                raise ValueError("Replay speed must be finite")
            self._speed = min(max(requested_speed, 0.1), 20.0)
            self._reset_anchor()

    def tick(self) -> None:
        with self._lock:
            if not self._playing or not self._samples:
                return
            elapsed_real_s = time.monotonic() - self._anchor_monotonic
            target_s = min(
                self._duration_s,
                self._anchor_playhead_s + elapsed_real_s * self._speed,
            )
            self._emit_until(target_s)
            self._playhead_s = target_s
            if target_s >= self._duration_s:
                self._playing = False
                self._reset_anchor()
                self.store.set_status("replay_complete", f"Replay complete: {self._name}")

    def status(self) -> dict[str, object]:
        with self._lock:
            return {
                "loaded": bool(self._samples),
                "name": self._name,
                "playing": self._playing,
                "playhead_s": self._playhead_s,
                "duration_s": self._duration_s,
                "speed": self._speed,
                "index": self._index,
                "count": len(self._samples),
                "launch_known": self._launch_known,
            }


class SourceManager:
    """Switches cleanly between a live/demo source and CSV replay."""

    def __init__(
        self,
        store: TelemetryStore,
        live_source_factory: Callable[[], threading.Thread],
        live_label: str,
        live_kind: str = "live",
        serial_port: Optional[str] = None,
        baud_rate: Optional[int] = None,
        initial_mode: str = "live",
    ) -> None:
        if live_kind not in {"live", "demo", "idle"}:
            raise ValueError("Live source kind must be 'live', 'demo', or 'idle'")
        if initial_mode not in {"live", "idle"}:
            raise ValueError("Initial source mode must be 'live' or 'idle'")
        if (serial_port is None) != (baud_rate is None):
            raise ValueError("Serial port and baud rate must be configured together")
        if serial_port is not None and baud_rate is not None:
            serial_port, baud_rate = validate_serial_settings(serial_port, baud_rate)
        self.store = store
        self.live_source_factory = live_source_factory
        self.live_label = live_label
        self.live_kind = live_kind
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.replay = ReplayController(store)
        self._lock = threading.RLock()
        self._live_source: Optional[threading.Thread] = None
        self._mode = initial_mode

    def _stop_live(self) -> bool:
        source = self._live_source
        if source is None:
            return True
        stop = getattr(source, "stop", None)
        if callable(stop):
            stop()
        if source.is_alive() and source is not threading.current_thread():
            source.join(timeout=2.0)
        if source.is_alive():
            self.store.set_status(
                "error",
                "Live source did not stop within 2 seconds",
            )
            return False
        self._live_source = None
        return True

    def start_live(self) -> None:
        with self._lock:
            try:
                new_source = self.live_source_factory()
            except Exception as exc:
                raise RuntimeError("Live source could not be created") from exc
            if new_source is None or not callable(getattr(new_source, "start", None)):
                raise RuntimeError("Live source factory did not return a startable source")
            if not self._stop_live():
                raise RuntimeError("Live source is still stopping")
            previous_mode = self._mode
            previous_replay_status = self.replay.status()
            self.store.stop_recording()
            # A replay and a live receiver are separate sessions. Returning to
            # live must not revive the pre-replay flight or join it to the next
            # receiver fix; previously recorded export points remain intact.
            self.store.clear_track(reset_counters=True)
            self.store.reset_sequence_tracking()
            self._mode = "live"
            self._live_source = new_source
            try:
                new_source.start()
            except Exception as exc:
                try:
                    cleanup_succeeded = self._stop_live()
                except Exception:
                    cleanup_succeeded = False
                if previous_mode == "replay" and cleanup_succeeded:
                    self._mode = "replay"
                    self.replay.seek(float(previous_replay_status["playhead_s"]))
                    self.store.set_status(
                        "error",
                        f"Live source failed to start; replay restored: {exc}",
                    )
                else:
                    self._mode = "live"
                    self.store.set_status("error", f"Live source failed to start: {exc}")
                raise RuntimeError("Live source failed to start") from exc
            self.replay.unload()

    def load_replay(self, samples: list[ReplaySample], name: str) -> None:
        prepared = self.replay.prepare_samples(samples)
        with self._lock:
            if not self._stop_live():
                raise RuntimeError("Live source is still stopping")
            self.store.stop_recording()
            self.replay.load_prepared(prepared, name)
            self._mode = "replay"

    def tick(self) -> None:
        with self._lock:
            if self._mode == "replay":
                self.replay.tick()

    def toggle_replay(self) -> bool:
        with self._lock:
            if self._mode != "replay":
                return False
            return self.replay.toggle()

    def toggle_recording(self) -> bool:
        """Toggle capture only while the live source owns the store.

        Keeping the mode check and store mutation under the source-manager lock
        prevents a concurrent replay upload from re-enabling recording after the
        source has already switched to replay mode.
        """
        with self._lock:
            if self._mode != "live":
                self.store.stop_recording()
                return False
            _, source_status = self.store.snapshot()
            if source_status["recording"]:
                self.store.stop_recording()
                return False
            # Recording is deliberately opt-in: points received before this
            # click remain in the live display, but never enter the CSV capture.
            self.store.start_recording(include_existing=False)
            return True

    def configure_serial(
        self,
        serial_port: object,
        baud_rate: object,
    ) -> tuple[str, int]:
        """Apply serial settings and explicitly reconnect the live source."""
        normalized_port, normalized_baud = validate_serial_settings(
            serial_port,
            baud_rate,
        )
        with self._lock:
            if self.live_kind != "live":
                raise RuntimeError("Select Real flight before connecting a serial port")
            if self.serial_port is None or self.baud_rate is None:
                raise RuntimeError("Serial controls are unavailable for this source")
            previous_factory = self.live_source_factory
            previous_label = self.live_label
            previous_port = self.serial_port
            previous_baud = self.baud_rate
            self.live_source_factory = (
                lambda port=normalized_port, baud=normalized_baud: SerialReceiver(
                    self.store,
                    port,
                    baud,
                )
            )
            try:
                self.start_live()
            except Exception:
                self.live_source_factory = previous_factory
                self.live_label = previous_label
                self.serial_port = previous_port
                self.baud_rate = previous_baud
                raise
            self.live_label = (
                f"Serial source / {normalized_port} / {normalized_baud} baud"
            )
            self.live_kind = "live"
            self.serial_port = normalized_port
            self.baud_rate = normalized_baud
            return normalized_port, normalized_baud

    def select_serial_mode(
        self,
        serial_port: object,
        baud_rate: object,
    ) -> tuple[str, int]:
        """Prepare real-flight mode without opening the port before Connect."""
        normalized_port, normalized_baud = validate_serial_settings(
            serial_port,
            baud_rate,
        )
        with self._lock:
            if not self._stop_live():
                raise RuntimeError("Live source is still stopping")
            self.store.stop_recording()
            self.store.clear_track(reset_counters=True)
            self.store.reset_sequence_tracking()
            self.replay.unload()
            port = normalized_port
            baud = normalized_baud
            self.serial_port = port
            self.baud_rate = baud
            self.live_source_factory = (
                lambda selected_port=port, selected_baud=baud: SerialReceiver(
                    self.store,
                    selected_port,
                    selected_baud,
                )
            )
            self.live_label = "Real flight / select a COM port and Connect"
            self.live_kind = "live"
            self._mode = "idle"
            self.store.set_status(
                "waiting",
                "Real flight selected · choose a COM port and baud rate, then Connect",
            )
            return port, baud

    def activate_demo(self) -> None:
        """Switch to a fresh synthetic flight without losing serial settings."""
        with self._lock:
            previous_factory = self.live_source_factory
            previous_label = self.live_label
            previous_kind = self.live_kind
            self.live_source_factory = create_demo_source_factory(self.store)
            try:
                self.start_live()
            except Exception:
                self.live_source_factory = previous_factory
                self.live_label = previous_label
                self.live_kind = previous_kind
                raise
            self.live_label = "DEMO source / synthetic 78-byte HYI packet model"
            self.live_kind = "demo"

    def seek_replay(self, playhead_s: float) -> None:
        with self._lock:
            if self._mode == "replay":
                self.replay.seek(playhead_s)

    def set_replay_speed(self, speed: float) -> None:
        with self._lock:
            if self._mode == "replay":
                self.replay.set_speed(speed)

    def status(self) -> dict[str, object]:
        with self._lock:
            status = self.replay.status()
            status["mode"] = self._mode
            status["live_kind"] = self.live_kind
            status["serial_configurable"] = (
                self.serial_port is not None and self.baud_rate is not None
            )
            status["serial_port"] = self.serial_port
            status["baud_rate"] = self.baud_rate
            status["source_selected"] = self.live_kind in {"live", "demo"}
            status["source_label"] = (
                f"CSV replay / {status['name']}"
                if self._mode == "replay"
                else self.live_label
            )
            return status

    def stop(self) -> None:
        with self._lock:
            self._stop_live()
            self.replay.pause()


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


def balanced_scene_aspect_ratio(
    east: list[float],
    north: list[float],
    up: list[float],
    minimum_ratio: float = MIN_SCENE_AXIS_RATIO,
) -> dict[str, float]:
    """Keep low-variation ENU axes visible without hiding their real ranges.

    Plotly's ``aspectmode='data'`` can collapse a nearly stationary horizontal
    axis into a paper-thin plane. The axis ticks still carry the true metre
    ranges; this ratio only prevents the 3D scene box from becoming unusable.
    """
    if not 0.0 < minimum_ratio <= 1.0:
        raise ValueError("minimum_ratio must be within (0, 1]")

    ranges = (
        padded_range(east),
        padded_range(north),
        padded_range(up),
    )
    spans = [axis_range[1] - axis_range[0] for axis_range in ranges]
    largest_span = max(spans)
    if largest_span <= 0.0:
        return {"x": 1.0, "y": 1.0, "z": 1.0}
    return {
        axis: max(minimum_ratio, span / largest_span)
        for axis, span in zip(("x", "y", "z"), spans)
    }


def _downsample_for_display(items: list, limit: int = MAX_RENDER_POINTS) -> list:
    """Select evenly spaced display points while retaining both endpoints."""
    if len(items) <= limit:
        return list(items)
    scale = (len(items) - 1) / (limit - 1)
    return [items[round(index * scale)] for index in range(limit)]


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
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2")
    progress = [index / (sample_count - 1) for index in range(sample_count)]
    east = [80.0 * value for value in progress]
    north = [18.0 * math.sin(math.pi * value) + 6.0 * value for value in progress]
    up = [120.0 * math.sin(math.pi * value) ** 1.25 for value in progress]
    return east, north, up


def add_representative_trajectory(
    figure: go.Figure,
    palette: dict[str, str],
    trajectory_points: Optional[list[TrajectoryOverlayPoint]] = None,
    trajectory_name: str = "Built-in reference",
):
    if trajectory_points:
        all_east = [point.east_m for point in trajectory_points]
        all_north = [point.north_m for point in trajectory_points]
        all_up = [point.up_m for point in trajectory_points]
        displayed = _downsample_for_display(trajectory_points)
        east = [point.east_m for point in displayed]
        north = [point.north_m for point in displayed]
        up = [point.up_m for point in displayed]
    else:
        east, north, up = representative_trajectory_coordinates()
        all_east, all_north, all_up = east, north, up
    figure.add_trace(
        go.Scatter3d(
            x=east,
            y=north,
            z=up,
            mode="lines",
            name=trajectory_name,
            line={
                "color": palette["reference"],
                "width": 6,
                "dash": "dash",
            },
            hovertemplate=(
                "Reference path<br>"
                "East: %{x:.1f} m<br>"
                "North: %{y:.1f} m<br>"
                "Up: %{z:.1f} m"
                "<extra></extra>"
            ),
        )
    )
    return all_east, all_north, all_up


def apply_trajectory_layout(
    figure: go.Figure,
    palette: dict[str, str],
    east: list[float],
    north: list[float],
    up: list[float],
    camera_revision: int = 0,
) -> None:
    axis_style = {
        "showbackground": True,
        "backgroundcolor": palette["axis"],
        "gridcolor": palette["grid"],
        "zerolinecolor": palette["zero"],
        "tickfont": {"color": palette["muted"]},
        "showspikes": False,
    }

    east_range = padded_range(east)
    north_range = padded_range(north)
    up_range = padded_range(up)
    aspect_ratio = balanced_scene_aspect_ratio(east, north, up)

    figure.update_layout(
        uirevision=f"trajectory-camera-{int(camera_revision)}",
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
                "range": east_range,
            },
            "yaxis": {
                **axis_style,
                "title": {
                    "text": "North (m)",
                    "font": {"color": palette["text"]},
                },
                "range": north_range,
            },
            "zaxis": {
                **axis_style,
                "title": {
                    "text": "Up (m)",
                    "font": {"color": palette["text"]},
                },
                "range": up_range,
            },
            "aspectmode": "manual",
            "aspectratio": aspect_ratio,
            "dragmode": "turntable",
            "camera": {
                "eye": {"x": 1.45, "y": 1.55, "z": 1.1},
                "up": {"x": 0.0, "y": 0.0, "z": 1.0},
                "center": {"x": 0.0, "y": 0.0, "z": 0.0},
                "projection": {"type": "perspective"},
            },
        },
    )


def add_rocket_indicator(figure: go.Figure, points: list[TelemetryPoint]) -> None:
    """Add a direction-aware 3D vehicle glyph at the current fix."""
    current = points[-1]
    if len(points) >= 2:
        previous = points[-2]
        direction = (
            current.east_m - previous.east_m,
            current.north_m - previous.north_m,
            current.up_m - previous.up_m,
        )
    else:
        direction = (0.0, 0.0, 1.0)
    magnitude = math.sqrt(sum(component * component for component in direction))
    if magnitude < 1e-6:
        direction = (0.0, 0.0, 1.0)
        magnitude = 1.0
    unit = tuple(component / magnitude for component in direction)
    span = max(
        20.0,
        max(point.east_m for point in points) - min(point.east_m for point in points),
        max(point.north_m for point in points) - min(point.north_m for point in points),
        max(point.up_m for point in points) - min(point.up_m for point in points),
    )
    glyph_size = min(30.0, max(4.0, span * 0.045))
    figure.add_trace(
        go.Cone(
            x=[current.east_m],
            y=[current.north_m],
            z=[current.up_m],
            u=[unit[0]],
            v=[unit[1]],
            w=[unit[2]],
            anchor="tail",
            sizemode="absolute",
            sizeref=glyph_size,
            colorscale=[[0.0, "#ffb35c"], [0.45, "#ff5f6d"], [1.0, "#f7fbff"]],
            showscale=False,
            showlegend=True,
            name="Rocket",
            hovertemplate="Vehicle direction<extra></extra>",
        )
    )


def create_trajectory_figure(
    points: list[TelemetryPoint],
    theme: str = "dark",
    show_reference: bool = False,
    trajectory_points: Optional[list[TrajectoryOverlayPoint]] = None,
    trajectory_name: str = "Built-in reference",
    launch_point: Optional[TelemetryPoint] = None,
    camera_revision: int = 0,
    launch_label: str = "Launch",
    reference_available: bool = True,
    reference_notice: Optional[str] = None,
) -> go.Figure:
    palette = FIGURE_THEMES.get(theme, FIGURE_THEMES["dark"])
    figure = go.Figure()

    if not points:
        if show_reference:
            east, north, up = add_representative_trajectory(
                figure, palette, trajectory_points, trajectory_name
            )
            apply_trajectory_layout(
                figure,
                palette,
                east,
                north,
                up,
                camera_revision,
            )
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
    step = max(1, math.ceil(len(points) / max_plot_points))
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

    actual_launch = launch_point or points[0]
    figure.add_trace(
        go.Scatter3d(
            x=[actual_launch.east_m],
            y=[actual_launch.north_m],
            z=[actual_launch.up_m],
            mode="markers",
            name=launch_label,
            marker={"size": 7, "color": "#57d3a2", "symbol": "diamond"},
            hovertemplate=f"{launch_label}<extra></extra>",
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

    add_rocket_indicator(figure, points)

    range_east = list(east)
    range_north = list(north)
    range_up = list(up)
    range_east.append(actual_launch.east_m)
    range_north.append(actual_launch.north_m)
    range_up.append(actual_launch.up_m)
    if show_reference and reference_available:
        reference_east, reference_north, reference_up = (
            add_representative_trajectory(
                figure, palette, trajectory_points, trajectory_name
            )
        )
        range_east.extend(reference_east)
        range_north.extend(reference_north)
        range_up.extend(reference_up)
    elif show_reference and reference_notice:
        figure.add_annotation(
            text=reference_notice,
            x=0.5,
            y=0.98,
            xref="paper",
            yref="paper",
            xanchor="center",
            yanchor="top",
            showarrow=False,
            font={"size": 12, "color": "#ffcf70"},
            bgcolor=palette["legend"],
            bordercolor=palette["border"],
            borderwidth=1,
            borderpad=8,
        )

    apply_trajectory_layout(
        figure,
        palette,
        range_east,
        range_north,
        range_up,
        camera_revision,
    )
    return figure


def _longitude_span_and_center(longitude: list[float]) -> tuple[float, float]:
    """Return the shortest longitude span and its center across the dateline."""
    if not longitude:
        return 0.0, 0.0
    wrapped = sorted(float(value) % 360.0 for value in longitude)
    if len(wrapped) == 1:
        center = ((wrapped[0] + 180.0) % 360.0) - 180.0
        return 0.0, center

    gaps = [
        wrapped[index + 1] - wrapped[index]
        for index in range(len(wrapped) - 1)
    ]
    gaps.append(wrapped[0] + 360.0 - wrapped[-1])
    largest_gap_index = max(range(len(gaps)), key=gaps.__getitem__)
    span = 360.0 - gaps[largest_gap_index]
    start = wrapped[(largest_gap_index + 1) % len(wrapped)]
    center_wrapped = (start + span / 2.0) % 360.0
    center = ((center_wrapped + 180.0) % 360.0) - 180.0
    return span, center


def _unwrap_longitudes(longitude: list[float]) -> list[float]:
    """Keep adjacent map segments on the short side of the antimeridian."""
    if not longitude:
        return []
    unwrapped = [float(longitude[0])]
    for raw_value in longitude[1:]:
        value = float(raw_value)
        while value - unwrapped[-1] > 180.0:
            value -= 360.0
        while value - unwrapped[-1] < -180.0:
            value += 360.0
        unwrapped.append(value)
    return unwrapped


def _mercator_y(latitude_deg: float) -> float:
    """Return the normalized Web Mercator Y coordinate for a latitude."""
    latitude = min(
        max(float(latitude_deg), -WEB_MERCATOR_MAX_LATITUDE_DEG),
        WEB_MERCATOR_MAX_LATITUDE_DEG,
    )
    return (1.0 - math.asinh(math.tan(math.radians(latitude))) / math.pi) / 2.0


def _latitude_span_and_center(latitude: list[float]) -> tuple[float, float]:
    """Return Web Mercator Y span and its visually centered latitude."""
    if not latitude:
        return 0.0, 0.0
    mercator_y = [_mercator_y(value) for value in latitude]
    low = min(mercator_y)
    high = max(mercator_y)
    center_y = (low + high) / 2.0
    center_latitude = math.degrees(
        math.atan(math.sinh(math.pi * (1.0 - 2.0 * center_y)))
    )
    return high - low, center_latitude


def _map_zoom(latitude: list[float], longitude: list[float]) -> float:
    if len(latitude) < 2 or len(longitude) < 2:
        return 15.0
    longitude_span, _ = _longitude_span_and_center(longitude)
    latitude_span, _ = _latitude_span_and_center(latitude)
    world_fraction = max(latitude_span, longitude_span / 360.0, 1e-12)
    zoom = math.log2(
        (MAP_FIT_VIEWPORT_PX * MAP_FIT_FRACTION)
        / (MAP_TILE_SIZE_PX * world_fraction)
    )
    return min(max(zoom, 0.0), 16.5)


def terrain_map_style(theme: str = "dark") -> dict[str, object]:
    """Return a token-free MapLibre style with OSM imagery and 3D DEM terrain."""
    dark = theme != "light"
    raster_paint: dict[str, float] = {"raster-opacity": 0.94}
    hillshade_paint: dict[str, object] = {
        "hillshade-exaggeration": 0.42,
        "hillshade-shadow-color": "#07111d" if dark else "#526170",
        "hillshade-highlight-color": "#70a9b9" if dark else "#ffffff",
        "hillshade-accent-color": "#214a5e" if dark else "#8aa0ad",
    }
    if dark:
        raster_paint.update(
            {
                "raster-saturation": -0.68,
                "raster-contrast": 0.18,
                "raster-brightness-min": 0.08,
                "raster-brightness-max": 0.48,
            }
        )

    return {
        "version": 8,
        "sources": {
            "osm": {
                "type": "raster",
                "tiles": [OSM_TILE_URL],
                "tileSize": 256,
                "maxzoom": 19,
                "attribution": "© OpenStreetMap contributors",
            },
            "terrain-dem": {
                "type": "raster-dem",
                "url": MAP_TERRAIN_TILEJSON_URL,
                "tileSize": 256,
            },
            "hillshade-dem": {
                "type": "raster-dem",
                "url": MAP_TERRAIN_TILEJSON_URL,
                "tileSize": 256,
            },
        },
        "layers": [
            {
                "id": "map-background",
                "type": "background",
                "paint": {
                    "background-color": "#07111d" if dark else "#dfeaf1"
                },
            },
            {
                "id": "osm-basemap",
                "type": "raster",
                "source": "osm",
                "paint": raster_paint,
            },
            {
                "id": "terrain-hillshade",
                "type": "hillshade",
                "source": "hillshade-dem",
                "paint": hillshade_paint,
            },
        ],
        "terrain": {"source": "terrain-dem", "exaggeration": 1.35},
    }


def create_map_figure(
    points: list[TelemetryPoint],
    theme: str = "dark",
    trajectory_points: Optional[list[TrajectoryOverlayPoint]] = None,
    launch_point: Optional[TelemetryPoint] = None,
    terrain_enabled: bool = True,
    launch_label: str = "Launch",
    include_builtin_reference: bool = False,
    trajectory_name: str = "Reference",
    ground_station: Optional[GroundStationLocation] = None,
    empty_state_text: Optional[str] = "Konum verisi bekleniyor…",
) -> go.Figure:
    """Create a token-free OpenStreetMap ground-track and terrain view."""
    palette = FIGURE_THEMES.get(theme, FIGURE_THEMES["dark"])
    figure = go.Figure()
    if points:
        newest_point = points[-1]
        map_data_revision = (
            f"telemetry-{len(points)}-{newest_point.time_s:.6f}-"
            f"{newest_point.latitude_deg:.8f}-{newest_point.longitude_deg:.8f}"
        )
    else:
        map_data_revision = "telemetry-empty"

    map_reference_points = list(trajectory_points or [])
    if include_builtin_reference and not map_reference_points:
        reference_east, reference_north, reference_up = (
            representative_trajectory_coordinates()
        )
        map_reference_points = [
            TrajectoryOverlayPoint(None, east_m, north_m, up_m)
            for east_m, north_m, up_m in zip(
                reference_east,
                reference_north,
                reference_up,
            )
        ]

    reference_anchor = launch_point or (points[0] if points else None)
    if reference_anchor is not None:
        anchor_latitude_rad = math.radians(reference_anchor.latitude_deg)
        anchor_longitude_rad = math.radians(reference_anchor.longitude_deg)
        projected_reference: list[TrajectoryOverlayPoint] = []
        for point in map_reference_points:
            if point.latitude_deg is not None and point.longitude_deg is not None:
                projected_reference.append(point)
                continue

            ground_distance_m = math.hypot(point.east_m, point.north_m)
            if ground_distance_m <= 1e-9:
                latitude_deg = reference_anchor.latitude_deg
                longitude_deg = reference_anchor.longitude_deg
            else:
                bearing_rad = math.atan2(point.east_m, point.north_m)
                angular_distance = ground_distance_m / EARTH_RADIUS_M
                latitude_rad = math.asin(
                    math.sin(anchor_latitude_rad) * math.cos(angular_distance)
                    + math.cos(anchor_latitude_rad)
                    * math.sin(angular_distance)
                    * math.cos(bearing_rad)
                )
                longitude_rad = anchor_longitude_rad + math.atan2(
                    math.sin(bearing_rad)
                    * math.sin(angular_distance)
                    * math.cos(anchor_latitude_rad),
                    math.cos(angular_distance)
                    - math.sin(anchor_latitude_rad) * math.sin(latitude_rad),
                )
                latitude_deg = math.degrees(latitude_rad)
                longitude_deg = (
                    (math.degrees(longitude_rad) + 180.0) % 360.0
                ) - 180.0
            projected_reference.append(
                TrajectoryOverlayPoint(
                    point.time_s,
                    point.east_m,
                    point.north_m,
                    point.up_m,
                    latitude_deg,
                    longitude_deg,
                    reference_anchor.altitude_m + point.up_m,
                )
            )
        map_reference_points = projected_reference

    reference_llh = [
        point
        for point in map_reference_points
        if point.latitude_deg is not None and point.longitude_deg is not None
    ]
    extent_latitude: list[float] = []
    extent_longitude: list[float] = []

    if points:
        plotted = _downsample_for_display(points)
        current = plotted[-1]
        latitude = [point.latitude_deg for point in plotted]
        longitude = [point.longitude_deg for point in plotted]
        customdata = [
            [
                point.altitude_m,
                point.up_m,
                point.time_s,
                point.longitude_deg,
            ]
            for point in plotted
        ]
        figure.add_trace(
            go.Scattermap(
                lat=latitude,
                lon=_unwrap_longitudes(longitude),
                mode="lines+markers",
                name="Ground track",
                line={"color": "#ff9f43", "width": 4},
                marker={
                    "size": 5,
                    "color": [point.up_m for point in plotted],
                    "colorscale": "Turbo",
                    "showscale": False,
                },
                customdata=customdata,
                hovertemplate=(
                    "Lat: %{lat:.7f}<br>Lon: %{customdata[3]:.7f}<br>"
                    "GNSS alt: %{customdata[0]:.1f} m<br>"
                    "Relative up: %{customdata[1]:.1f} m<br>"
                    "Time: %{customdata[2]:.1f} s<extra></extra>"
                ),
            )
        )
        launch = launch_point or plotted[0]
        figure.add_trace(
            go.Scattermap(
                lat=[launch.latitude_deg],
                lon=[launch.longitude_deg],
                mode="markers",
                name=launch_label,
                marker={"size": 13, "color": "#57d3a2"},
                hovertemplate=f"{launch_label}<extra></extra>",
            )
        )
        figure.add_trace(
            go.Scattermap(
                lat=[current.latitude_deg],
                lon=[current.longitude_deg],
                mode="markers+text",
                name="Current",
                text=[f"🚀 {current.up_m:+,.0f} m"],
                textposition="top center",
                textfont={"color": palette["text"], "size": 13},
                marker={"size": 20, "color": "#ff4d6d"},
                customdata=[
                    [
                        current.longitude_deg,
                        current.altitude_m,
                        current.up_m,
                        current.time_s,
                    ]
                ],
                hovertemplate=(
                    "Current position<br>Lat: %{lat:.7f}<br>"
                    "Lon: %{customdata[0]:.7f}<br>"
                    "GNSS alt: %{customdata[1]:.1f} m<br>"
                    "Relative up: %{customdata[2]:.1f} m<br>"
                    "Time: %{customdata[3]:.1f} s<extra></extra>"
                ),
            )
        )
        center = {"lat": latitude[-1], "lon": longitude[-1]}
        extent_latitude.extend(point.latitude_deg for point in points)
        extent_longitude.extend(point.longitude_deg for point in points)
        extent_latitude.append(launch.latitude_deg)
        extent_longitude.append(launch.longitude_deg)
    elif reference_llh:
        reference_latitude = [float(point.latitude_deg) for point in reference_llh]
        reference_longitude = [float(point.longitude_deg) for point in reference_llh]
        _, reference_center_lon = _longitude_span_and_center(reference_longitude)
        _, reference_center_lat = _latitude_span_and_center(reference_latitude)
        center = {
            "lat": reference_center_lat,
            "lon": reference_center_lon,
        }
        extent_latitude.extend(reference_latitude)
        extent_longitude.extend(reference_longitude)
        zoom = _map_zoom(extent_latitude, extent_longitude)
    elif launch_point is not None:
        center = {
            "lat": launch_point.latitude_deg,
            "lon": launch_point.longitude_deg,
        }
        zoom = 14.5
    elif ground_station is not None:
        center = {
            "lat": ground_station.latitude_deg,
            "lon": ground_station.longitude_deg,
        }
        zoom = 14.5
    else:
        center = {"lat": 40.962105, "lon": 29.129418}
        zoom = 14.5

    if not points and not reference_llh and empty_state_text:
        figure.add_annotation(
            text=empty_state_text,
            x=0.5,
            y=0.5,
            xref="paper",
            yref="paper",
            showarrow=False,
            font={"color": palette["muted"], "size": 16},
            bgcolor=palette["legend"],
            borderpad=10,
        )

    if len(reference_llh) >= 2:
        reference_latitude = [float(point.latitude_deg) for point in reference_llh]
        reference_longitude = [float(point.longitude_deg) for point in reference_llh]
        displayed_reference = _downsample_for_display(reference_llh)
        displayed_reference_latitude = [
            float(point.latitude_deg) for point in displayed_reference
        ]
        displayed_reference_longitude = [
            float(point.longitude_deg) for point in displayed_reference
        ]
        figure.add_trace(
            go.Scattermap(
                lat=displayed_reference_latitude,
                lon=_unwrap_longitudes(displayed_reference_longitude),
                mode="lines",
                name=trajectory_name,
                line={"color": palette["reference"], "width": 3},
                hovertemplate=f"{trajectory_name}<extra></extra>",
            )
        )
        extent_latitude.extend(reference_latitude)
        extent_longitude.extend(reference_longitude)

    if ground_station is not None:
        station_altitude = (
            "--"
            if ground_station.altitude_m is None
            else f"{ground_station.altitude_m:.1f} m"
        )
        station_accuracy = (
            "--"
            if ground_station.accuracy_m is None
            else f"±{ground_station.accuracy_m:.0f} m"
        )
        figure.add_trace(
            go.Scattermap(
                lat=[ground_station.latitude_deg],
                lon=[ground_station.longitude_deg],
                mode="markers+text",
                name="Ground station",
                text=["GS"],
                textposition="top center",
                marker={"size": 17, "color": "#67a1ff"},
                customdata=[
                    [
                        ground_station.longitude_deg,
                        station_altitude,
                        station_accuracy,
                        ground_station.source,
                    ]
                ],
                hovertemplate=(
                    "Ground station<br>Lat: %{lat:.7f}<br>"
                    "Lon: %{customdata[0]:.7f}<br>"
                    "Altitude: %{customdata[1]}<br>"
                    "Accuracy: %{customdata[2]}<br>"
                    "Source: %{customdata[3]}<extra></extra>"
                ),
            )
        )
        extent_latitude.append(ground_station.latitude_deg)
        extent_longitude.append(ground_station.longitude_deg)

    if extent_latitude and extent_longitude:
        _, extent_center_lon = _longitude_span_and_center(extent_longitude)
        _, extent_center_lat = _latitude_span_and_center(extent_latitude)
        center = {
            "lat": extent_center_lat,
            "lon": extent_center_lon,
        }
        zoom = _map_zoom(extent_latitude, extent_longitude)

    if not any(trace.type == "scattermap" for trace in figure.data):
        # Plotly only instantiates its MapLibre subplot when at least one map
        # trace exists. Keep a transparent anchor so the terrain remains open
        # before the first fix and for single-point reference trajectories.
        figure.add_trace(
            go.Scattermap(
                lat=[center["lat"]],
                lon=[center["lon"]],
                mode="markers",
                marker={"size": 1, "opacity": 0.0},
                hoverinfo="skip",
                showlegend=False,
                name="Map anchor",
            )
        )

    figure.update_layout(
        # The lightweight browser-side stream callback changes trace arrays;
        # datarevision advertises a fresh coordinate set while uirevision
        # preserves the operator's chosen map camera.
        datarevision=map_data_revision,
        uirevision=f"keep-map-{'terrain' if terrain_enabled else 'flat'}",
        map={
            "style": (
                terrain_map_style(theme)
                if terrain_enabled
                else "open-street-map"
            ),
            "center": center,
            "zoom": zoom,
            "pitch": MAP_TERRAIN_PITCH_DEG if terrain_enabled else 0.0,
            "bearing": MAP_TERRAIN_BEARING_DEG if terrain_enabled else 0.0,
        },
        paper_bgcolor=palette["panel"],
        plot_bgcolor=palette["panel"],
        font={"family": "Inter, Segoe UI, Arial", "color": palette["text"]},
        margin={"l": 0, "r": 0, "t": 0, "b": 0},
        legend={
            "x": 0.02,
            "y": 0.98,
            "bgcolor": palette["legend"],
            "bordercolor": palette["border"],
            "borderwidth": 1,
        },
    )
    return figure


def create_profile_figure(
    points: list[TelemetryPoint],
    theme: str = "dark",
    trajectory_points: Optional[list[TrajectoryOverlayPoint]] = None,
) -> go.Figure:
    palette = FIGURE_THEMES.get(theme, FIGURE_THEMES["dark"])
    figure = go.Figure()
    if points:
        plotted = _downsample_for_display(points)
        figure.add_trace(
            go.Scatter(
                x=[point.time_s for point in plotted],
                y=[point.up_m for point in plotted],
                mode="lines",
                name="Measured",
                line={"color": "#ff9f43", "width": 3},
                fill="tozeroy",
                fillcolor="rgba(255,159,67,0.12)",
                hovertemplate="%{x:.1f} s / %{y:.1f} m<extra></extra>",
            )
        )
    timed_reference = [
        point for point in (trajectory_points or []) if point.time_s is not None
    ]
    if timed_reference:
        displayed_reference = _downsample_for_display(timed_reference)
        figure.add_trace(
            go.Scatter(
                x=[point.time_s for point in displayed_reference],
                y=[point.up_m for point in displayed_reference],
                mode="lines",
                name="Reference",
                line={"color": palette["reference"], "width": 2, "dash": "dash"},
                hovertemplate="%{x:.1f} s / %{y:.1f} m<extra></extra>",
            )
        )
    figure.update_layout(
        uirevision="keep-profile",
        paper_bgcolor=palette["panel"],
        plot_bgcolor=palette["panel"],
        font={"family": "Inter, Segoe UI, Arial", "color": palette["text"]},
        margin={"l": 46, "r": 12, "t": 8, "b": 36},
        xaxis={
            "title": "Flight time (s)",
            "gridcolor": palette["grid"],
            "zerolinecolor": palette["zero"],
        },
        yaxis={
            "title": "Relative up (m)",
            "gridcolor": palette["grid"],
            "zerolinecolor": palette["zero"],
        },
        legend={"orientation": "h", "x": 0.01, "y": 0.99},
    )
    return figure


def infer_flight_phase(points: list[TelemetryPoint], vertical_speed_mps: float) -> str:
    if not points:
        return "STANDBY"
    current = points[-1]
    if current.time_s < 2.0 and current.up_m < 3.0:
        return "PAD"
    if current.time_s > 8.0 and current.up_m < 3.0 and abs(vertical_speed_mps) < 2.0:
        return "LANDED"
    if vertical_speed_mps > 3.0:
        return "ASCENT"
    if vertical_speed_mps < -3.0:
        return "DESCENT"
    if current.up_m > 10.0:
        return "APOGEE / COAST"
    return "TRACKING"


def stat_card(
    label: str, value_id: str, unit: str = "", wide: bool = False
) -> html.Div:
    return html.Div(
        [
            html.Div(label, className="stat-label"),
            html.Div(
                [
                    html.Span("--", id=value_id, className="stat-value"),
                    html.Span(unit, className="stat-unit"),
                ],
                className="stat-value-row",
            ),
        ],
        className="stat-card" + (" stat-card-wide" if wide else ""),
    )


def _decode_dash_upload(contents: str) -> bytes:
    if not isinstance(contents, str) or not contents or "," not in contents:
        raise ValueError("Upload does not contain file data")
    metadata, encoded = contents.split(",", 1)
    if ";base64" not in metadata:
        raise ValueError("Upload encoding is not supported")
    max_encoded_chars = 4 * math.ceil(MAX_UPLOAD_BYTES / 3)
    if len(encoded) > max_encoded_chars:
        raise ValueError("Uploaded CSV exceeds the 16 MiB size limit")
    try:
        decoded = base64.b64decode(encoded, validate=True)
    except (ValueError, TypeError) as exc:
        raise ValueError("Uploaded file is not valid base64 data") from exc
    if len(decoded) > MAX_UPLOAD_BYTES:
        raise ValueError("Uploaded CSV exceeds the 16 MiB size limit")
    return decoded


def _format_clock(seconds: float) -> str:
    seconds = float(seconds)
    if not math.isfinite(seconds):
        return "--:--"
    seconds = max(0.0, seconds)
    minutes, remaining = divmod(seconds, 60.0)
    return f"{int(minutes):02d}:{remaining:04.1f}"


def map_empty_state_message(
    mode: object,
    live_kind: object,
    prompt_elapsed_s: float = 0.0,
) -> Optional[str]:
    """Describe why the map has no position without calling Demo a live GNSS source."""

    normalized_mode = str(mode or "idle").strip().casefold()
    normalized_kind = str(live_kind or "idle").strip().casefold()
    if normalized_mode == "live" and normalized_kind == "demo":
        return "Demo telemetrisi başlatılıyor…"
    if normalized_mode == "live" and normalized_kind == "live":
        return "İlk GNSS konumu bekleniyor…"
    if normalized_mode == "replay":
        return "Kayıt konumu bekleniyor…"
    if normalized_kind == "live":
        return "COM portunu seçip Connect'e bas"
    if prompt_elapsed_s >= MODE_SELECTION_PROMPT_TIMEOUT_S:
        return None
    return "Demo veya Gerçek Uçuş seç"


def build_dash_app(
    store: TelemetryStore,
    source_manager: SourceManager,
    trajectory_store: TrajectoryStore,
    stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S,
) -> Dash:
    app = Dash(__name__, update_title=None)
    app.title = APP_NAME
    app.server.config["MAX_CONTENT_LENGTH"] = MAX_REQUEST_BYTES
    live_dvr = LiveDvrController()

    graph_config = {
        "displaylogo": False,
        "scrollZoom": True,
        "responsive": True,
        "modeBarButtonsToRemove": ["select3d", "lasso3d"],
    }
    map_graph_config = {
        **graph_config,
        # External OSM/DEM tiles cannot be exported reliably by Plotly's
        # browser-side canvas snapshotter (CORS/MapLibre limitation).
        "modeBarButtonsToRemove": ["toImage", "select2d", "lasso2d"],
    }
    initial_manager_status = source_manager.status()
    serial_controls_active = (
        initial_manager_status["live_kind"] == "live"
        and initial_manager_status["mode"] in {"idle", "live"}
    )
    initial_serial_port = (
        initial_manager_status["serial_port"] or DEFAULT_SERIAL_PORT
    )
    initial_baud_rate = (
        initial_manager_status["baud_rate"] or DEFAULT_BAUD_RATE
    )
    initial_live_kind = str(initial_manager_status["live_kind"])
    initial_runtime_mode = str(initial_manager_status["mode"])
    initial_source_mode_label = (
        "SIMULATION"
        if initial_runtime_mode == "live" and initial_live_kind == "demo"
        else "LIVE"
        if initial_runtime_mode == "live" and initial_live_kind == "live"
        else "REPLAY"
        if initial_runtime_mode == "replay"
        else "SERIAL READY"
        if initial_live_kind == "live"
        else "CHOOSE MODE"
    )
    initial_source_message = (
        "Demo flight is active. Select Real flight whenever hardware is ready."
        if initial_live_kind == "demo"
        else "Real flight is active. Select a COM port and baud rate to reconnect."
        if initial_live_kind == "live"
        else "Choose Demo flight for a simulated mission or Real flight for COM telemetry."
    )

    app.layout = html.Div(
        [
            html.Div(
                [
                    html.Div(
                        [
                            html.Img(
                                src="/assets/rocket.svg",
                                className="brand-mark",
                                alt=APP_NAME,
                            ),
                            html.Div(
                                [
                                    html.Div("PROİST ROKET TAKIMI", className="brand-kicker"),
                                    html.H1(
                                        APP_NAME,
                                        className="brand-title",
                                    ),
                                    html.Div(
                                        source_manager.live_label,
                                        id="source-label",
                                        className="brand-subtitle",
                                    ),
                                ]
                            ),
                        ],
                        className="brand-lockup",
                    ),
                    html.Div(
                        [
                            html.Div(
                                [
                                    html.Span(className="status-dot"),
                                    html.Span("WAITING", id="source-status"),
                                ],
                                id="source-status-chip",
                                className="status-chip",
                                **{"data-status": "waiting"},
                            ),
                            html.Div(
                                initial_source_mode_label,
                                id="source-mode",
                                className="mode-pill",
                            ),
                            html.Div("REC OFF", id="record-indicator", className="mode-pill"),
                            html.Button(
                                "Start recording",
                                id="record-toggle",
                                n_clicks=0,
                                className="button button-primary",
                                disabled=initial_runtime_mode != "live",
                            ),
                            html.Button(
                                "Export CSV",
                                id="export-recording",
                                n_clicks=0,
                                className="button",
                            ),
                            html.Button(
                                [
                                    html.Span(
                                        "⌖",
                                        className="station-trigger-icon",
                                        **{"aria-hidden": "true"},
                                    ),
                                    html.Span("GS", className="station-trigger-label"),
                                ],
                                id="ground-station-toggle",
                                n_clicks=0,
                                className="button station-popup-trigger",
                                title="Yer istasyonu konumunu ayarla",
                                **{
                                    "aria-controls": "ground-station-popover",
                                    "aria-haspopup": "dialog",
                                    "aria-label": "Yer istasyonu",
                                },
                            ),
                            html.Button(
                                "Light mode",
                                id="theme-toggle",
                                n_clicks=0,
                                className="button",
                            ),
                            html.Button(
                                "Fullscreen",
                                id="fullscreen-toggle",
                                n_clicks=0,
                                className="button fullscreen-button",
                                title="Fill the screen (Esc to exit)",
                                **{"aria-pressed": "false"},
                            ),
                        ],
                        className="header-actions",
                    ),
                ],
                className="topbar",
            ),
            html.Div(
                [
                    html.Div(
                        [
                            html.Div("DATA SOURCE", className="source-choice-kicker"),
                            html.H2("Choose flight mode", className="source-choice-title"),
                            html.Div(
                                "One application and one dashboard; only the telemetry source changes.",
                                className="source-choice-subtitle",
                            ),
                        ],
                        className="source-choice-copy",
                    ),
                    html.Div(
                        [
                            html.Button(
                                [
                                    html.Span("DEMO", className="source-choice-tag"),
                                    html.Span("Demo flight", className="source-choice-name"),
                                    html.Span(
                                        "Run a synthetic mission without hardware",
                                        className="source-choice-description",
                                    ),
                                ],
                                id="source-demo",
                                n_clicks=0,
                                className=(
                                    "source-choice-button source-choice-active"
                                    if initial_live_kind == "demo"
                                    else "source-choice-button"
                                ),
                            ),
                            html.Button(
                                [
                                    html.Span("SERIAL", className="source-choice-tag"),
                                    html.Span("Real flight", className="source-choice-name"),
                                    html.Span(
                                        "Receive live telemetry from the selected COM port",
                                        className="source-choice-description",
                                    ),
                                ],
                                id="source-live",
                                n_clicks=0,
                                className=(
                                    "source-choice-button source-choice-active"
                                    if initial_live_kind == "live"
                                    else "source-choice-button"
                                ),
                            ),
                        ],
                        className="source-choice-actions",
                    ),
                    html.Div(
                        initial_source_message,
                        id="source-choice-message",
                        className="source-choice-message",
                    ),
                ],
                className="source-choice-panel",
            ),
            html.Div(
                [
                    html.Div(
                        [
                            html.Span("VIEW", className="control-label"),
                            dcc.RadioItems(
                                id="view-mode",
                                options=[
                                    {"label": "3D Map", "value": "map"},
                                    {
                                        "label": "3D Map + Flight",
                                        "value": "split",
                                    },
                                    {"label": "Flight 3D", "value": "3d"},
                                ],
                                value="split",
                                inline=True,
                                className="segmented-control",
                            ),
                        ],
                        className="control-group",
                    ),
                    html.Div(
                        [
                            html.Button(
                                "Hide reference",
                                id="reference-path-toggle",
                                n_clicks=0,
                                className="button button-active",
                            ),
                            dcc.Upload(
                                id="trajectory-upload",
                                children=html.Span("Load trajectory"),
                                accept=".csv,text/csv",
                                multiple=False,
                                className="upload-box",
                            ),
                            html.Button(
                                "Clear trajectory",
                                id="clear-trajectory",
                                n_clicks=0,
                                className="button",
                            ),
                        ],
                        className="control-group",
                    ),
                    html.Div(
                        [
                            html.Span("SERIAL", className="control-label"),
                            dcc.Dropdown(
                                id="serial-port-select",
                                options=enumerate_serial_port_options(
                                    initial_serial_port
                                ),
                                value=initial_serial_port,
                                clearable=False,
                                searchable=True,
                                disabled=not serial_controls_active,
                                className="serial-select serial-port-select",
                            ),
                            html.Button(
                                "Ports ↻",
                                id="serial-refresh",
                                n_clicks=0,
                                className="button",
                                disabled=not serial_controls_active,
                                title="Refresh detected serial ports",
                            ),
                            dcc.Dropdown(
                                id="serial-baud-select",
                                options=serial_baud_options(initial_baud_rate),
                                value=initial_baud_rate,
                                clearable=False,
                                searchable=False,
                                disabled=not serial_controls_active,
                                className="serial-select serial-baud-select",
                            ),
                            html.Button(
                                "Connect",
                                id="serial-connect",
                                n_clicks=0,
                                className="button button-primary",
                                disabled=not serial_controls_active,
                            ),
                            html.Span(
                                (
                                    "Select settings, then Connect"
                                    if serial_controls_active
                                    else "Select Real flight to configure serial"
                                ),
                                id="serial-config-message",
                                className="control-hint",
                            ),
                        ],
                        id="serial-control-group",
                        className=(
                            "control-group serial-control-group"
                            if serial_controls_active
                            else "control-group serial-control-group serial-controls-hidden"
                        ),
                    ),
                    html.Div(
                        [
                            dcc.Upload(
                                id="replay-upload",
                                children=html.Span("Load replay CSV"),
                                accept=".csv,text/csv",
                                multiple=False,
                                className="upload-box",
                            ),
                            html.Button(
                                (
                                    "Demo active"
                                    if initial_runtime_mode == "live"
                                    and initial_live_kind == "demo"
                                    else "Live active"
                                    if initial_runtime_mode == "live"
                                    and initial_live_kind == "live"
                                    else "Return to demo"
                                    if initial_runtime_mode == "replay"
                                    and initial_live_kind == "demo"
                                    else "Return to live"
                                    if initial_runtime_mode == "replay"
                                    and initial_live_kind == "live"
                                    else "Real flight ready"
                                    if initial_live_kind == "live"
                                    else "Choose mode"
                                ),
                                id="return-live",
                                n_clicks=0,
                                className="button",
                                disabled=initial_runtime_mode != "replay",
                            ),
                            html.Button(
                                "Reset track",
                                id="reset-track",
                                n_clicks=0,
                                className="button button-danger",
                            ),
                        ],
                        className="control-group",
                    ),
                ],
                className="command-bar",
            ),
            html.Div(
                [
                    html.Div(
                        [
                            html.Div(
                                [
                                    html.Div(
                                        "YER İSTASYONU",
                                        className="ground-station-popover-kicker",
                                    ),
                                    html.Div(
                                        "Konum ayarları",
                                        className="ground-station-popover-title",
                                    ),
                                ]
                            ),
                            html.Button(
                                "×",
                                id="ground-station-close",
                                n_clicks=0,
                                className="ground-station-popover-close",
                                title="Pencereyi kapat",
                                **{"aria-label": "Yer istasyonu penceresini kapat"},
                            ),
                        ],
                        className="ground-station-popover-header",
                    ),
                    html.Div(
                        "Konum Windows üzerinden otomatik alınır. Alınamazsa "
                        "koordinatları elle girebilirsin.",
                        className="ground-station-popover-copy",
                    ),
                    html.Div(
                        [
                            html.Label(
                                [
                                    html.Span("Enlem", className="ground-station-label"),
                                    dcc.Input(
                                        id="ground-station-latitude",
                                        type="number",
                                        min=-90,
                                        max=90,
                                        step="any",
                                        placeholder="40.962105",
                                        debounce=True,
                                        className="ground-station-input",
                                    ),
                                ],
                                className="ground-station-field",
                            ),
                            html.Label(
                                [
                                    html.Span("Boylam", className="ground-station-label"),
                                    dcc.Input(
                                        id="ground-station-longitude",
                                        type="number",
                                        min=-180,
                                        max=180,
                                        step="any",
                                        placeholder="29.129418",
                                        debounce=True,
                                        className="ground-station-input",
                                    ),
                                ],
                                className="ground-station-field",
                            ),
                            html.Label(
                                [
                                    html.Span(
                                        "Rakım (m, isteğe bağlı)",
                                        className="ground-station-label",
                                    ),
                                    dcc.Input(
                                        id="ground-station-altitude",
                                        type="number",
                                        step="any",
                                        placeholder="0",
                                        debounce=True,
                                        className="ground-station-input",
                                    ),
                                ],
                                className="ground-station-field ground-station-field-wide",
                            ),
                        ],
                        className="ground-station-input-grid",
                    ),
                    html.Div(
                        [
                            html.Button(
                                "Bilgisayardan al",
                                id="ground-station-auto",
                                n_clicks=0,
                                className="button",
                            ),
                            html.Button(
                                "Konumu kaydet",
                                id="ground-station-apply",
                                n_clicks=0,
                                className="button button-primary",
                            ),
                        ],
                        className="ground-station-actions",
                    ),
                    html.Div(
                        "Bilgisayar konumu isteniyor…",
                        id="ground-station-status",
                        className="ground-station-status",
                    ),
                    html.Div(
                        "Haritada GS işaretiyle gösterilir.",
                        className="ground-station-map-note",
                    ),
                ],
                id="ground-station-popover",
                className="ground-station-popover",
                hidden=True,
                role="dialog",
                **{"aria-label": "Yer istasyonu konum ayarları"},
            ),
            html.Div(
                [
                    html.Div(
                        [
                            html.Div(
                                [
                                    html.Div(
                                        [
                                            html.Div(
                                                [
                                                    html.Div(
                                                        "3D flight corridor",
                                                        className="panel-title",
                                                    ),
                                                    html.Div(
                                                        "ENU trajectory, ground projection "
                                                        "and vehicle vector",
                                                        className="panel-subtitle",
                                                    ),
                                                ]
                                            ),
                                            html.Div(
                                                [
                                                    html.Div(
                                                        "WGS84 / LOCAL ENU",
                                                        className="mode-pill",
                                                    ),
                                                    html.Button(
                                                        "Reset 3D view",
                                                        id="reset-3d-view",
                                                        n_clicks=0,
                                                        className="button",
                                                    ),
                                                ],
                                                className="panel-actions",
                                            ),
                                        ],
                                        className="panel-header",
                                    ),
                                    dcc.Graph(
                                        id="trajectory-graph",
                                        figure=create_trajectory_figure([]),
                                        config=graph_config,
                                        className="graph-stage",
                                    ),
                                ],
                                id="trajectory-panel",
                                className="visual-panel",
                            ),
                            html.Div(
                                [
                                    html.Div(
                                        [
                                            html.Div(
                                                [
                                                    html.Div(
                                                        "3D terrain / ground track map",
                                                        className="panel-title",
                                                    ),
                                                    html.Div(
                                                        "Horizontal live/replay track · "
                                                        "rocket label shows relative altitude",
                                                        className="panel-subtitle",
                                                    ),
                                                ]
                                            ),
                                            html.Div(
                                                [
                                                    html.Div(
                                                        "OSM + DEM",
                                                        className="mode-pill",
                                                    ),
                                                    html.Button(
                                                        "Flat map",
                                                        id="terrain-toggle",
                                                        n_clicks=0,
                                                        className="button button-active",
                                                    ),
                                                ],
                                                className="panel-actions",
                                            ),
                                        ],
                                        className="panel-header",
                                    ),
                                    html.Div(
                                        [
                                            dcc.Graph(
                                                id="map-graph",
                                                figure=create_map_figure(
                                                    [],
                                                    terrain_enabled=True,
                                                    empty_state_text=map_empty_state_message(
                                                        initial_runtime_mode,
                                                        initial_live_kind,
                                                    ),
                                                ),
                                                config=map_graph_config,
                                                className="map-stage",
                                            ),
                                            html.Div(
                                                id="map-live-badge",
                                                className="map-live-badge",
                                            ),
                                        ],
                                        id="map-graph-host",
                                        className="map-graph-host",
                                    ),
                                ],
                                id="map-panel",
                                className="visual-panel",
                            ),
                            html.Div(
                                [
                                    html.Div(
                                        [
                                            html.Div(
                                                [
                                                    html.Div(
                                                        "Altitude profile",
                                                        className="panel-title",
                                                    ),
                                                    html.Div(
                                                        "Measured flight versus loaded reference",
                                                        className="panel-subtitle",
                                                    ),
                                                ]
                                            )
                                        ],
                                        className="panel-header",
                                    ),
                                    dcc.Graph(
                                        id="profile-graph",
                                        figure=create_profile_figure([]),
                                        config={
                                            "displaylogo": False,
                                            "responsive": True,
                                            "modeBarButtonsToRemove": ["select2d", "lasso2d"],
                                        },
                                        className="profile-stage",
                                    ),
                                ],
                                className="visual-panel",
                            ),
                        ],
                        className="visual-stack",
                    ),
                    html.Div(
                        [
                            html.Div(
                                [
                                    stat_card("Flight phase", "phase-value", wide=True),
                                    stat_card("Latitude", "latitude-value", "deg"),
                                    stat_card("Longitude", "longitude-value", "deg"),
                                    stat_card("GNSS altitude", "altitude-value", "m"),
                                    stat_card("Relative up", "up-value", "m"),
                                    stat_card("Ground speed", "ground-speed-value", "m/s"),
                                    stat_card("Vertical speed", "vertical-speed-value", "m/s"),
                                    stat_card("Ground range", "range-value", "m"),
                                    stat_card("Max relative up", "max-up-value", "m"),
                                    stat_card("Flight time", "flight-time-value", "s"),
                                    stat_card("Distance flown", "distance-value", "m"),
                                ],
                                className="metrics-grid",
                            ),
                            html.Div(
                                [
                                    html.Div(
                                        "TIMELINE / PLAYBACK",
                                        id="playback-title",
                                        className="info-title",
                                    ),
                                    html.Div(
                                        [
                                            html.Button(
                                                "Play",
                                                id="replay-play",
                                                n_clicks=0,
                                                className="button button-primary",
                                                disabled=True,
                                            ),
                                            html.Button(
                                                "Restart",
                                                id="replay-restart",
                                                n_clicks=0,
                                                className="button",
                                                disabled=True,
                                            ),
                                            dcc.Dropdown(
                                                id="replay-speed",
                                                options=[
                                                    {"label": "0.25×", "value": 0.25},
                                                    {"label": "0.5×", "value": 0.5},
                                                    {"label": "1×", "value": 1.0},
                                                    {"label": "2×", "value": 2.0},
                                                    {"label": "5×", "value": 5.0},
                                                    {"label": "10×", "value": 10.0},
                                                ],
                                                value=1.0,
                                                clearable=False,
                                                searchable=False,
                                                disabled=True,
                                                style={"width": "92px"},
                                            ),
                                        ],
                                        className="playback-row",
                                    ),
                                    html.Div(
                                        [
                                            html.Div(
                                                dcc.Slider(
                                                    id="replay-slider",
                                                    min=0.0,
                                                    max=0.1,
                                                    step=0.05,
                                                    value=0.0,
                                                    disabled=True,
                                                    marks={0: "0 s"},
                                                    updatemode="mouseup",
                                                    tooltip={
                                                        "placement": "bottom",
                                                        "always_visible": False,
                                                    },
                                                ),
                                                className="slider-wrap",
                                            ),
                                            html.Div(
                                                "00:00.0 / 00:00.0",
                                                id="replay-time",
                                                className="replay-time timeline-live",
                                            ),
                                        ],
                                        className="playback-row",
                                    ),
                                    html.Div(
                                        "No live source or replay",
                                        id="replay-file-name",
                                        className="info-value",
                                    ),
                                    html.Div(
                                        "Start Demo/Real flight or load a recorded CSV.",
                                        id="playback-message",
                                        className="panel-subtitle",
                                    ),
                                ],
                                className="playback-panel",
                            ),
                            html.Div(
                                [
                                    html.Div("PACKET HEALTH", className="info-title"),
                                    html.Div(
                                        [
                                            html.Span("0 accepted", id="accepted-value"),
                                            html.Span(" · "),
                                            html.Span("0 rejected", id="rejected-value"),
                                            html.Br(),
                                            html.Span("0 missing", id="missing-value"),
                                        ],
                                        className="info-value",
                                    ),
                                ],
                                className="info-card",
                            ),
                            html.Div(
                                [
                                    html.Div("SOURCE MESSAGE", className="info-title"),
                                    html.Div(
                                        "Waiting for source",
                                        id="source-message",
                                        className="info-value",
                                    ),
                                ],
                                className="info-card",
                            ),
                            html.Div(
                                [
                                    html.Div("REFERENCE TRAJECTORY", className="info-title"),
                                    html.Div(
                                        "Built-in reference",
                                        id="trajectory-name",
                                        className="info-value",
                                    ),
                                    html.Div(
                                        "Load OpenRocket or ENU/LLH CSV",
                                        id="trajectory-message",
                                        className="panel-subtitle",
                                    ),
                                ],
                                className="info-card",
                            ),
                            html.Div(
                                        "OSM and terrain tiles require internet; "
                                        "telemetry, replay and ENU 3D remain local.",
                                className="footer-note",
                            ),
                        ],
                        className="side-column",
                    ),
                ],
                className="workspace-grid",
            ),
            dcc.Store(id="theme-mode", storage_type="local", data="dark"),
            dcc.Store(id="reference-path-visible", data=True),
            dcc.Store(id="terrain-enabled", storage_type="local", data=True),
            dcc.Store(id="playback-revision", data={"revision": 0, "feedback": ""}),
            dcc.Store(id="recording-revision", data=0),
            dcc.Store(id="serial-revision", data=0),
            dcc.Store(
                id="source-mode-revision",
                data={"revision": 0, "feedback": ""},
            ),
            dcc.Store(
                id="ground-station-store",
                storage_type="session",
                data=None,
            ),
            dcc.Store(id="map-stream-data", data=None),
            dcc.Store(id="map-stream-applied", data=None),
            dcc.Store(id="rendered-playhead", data=0.0),
            dcc.Geolocation(
                id="ground-station-geolocation",
                update_now=True,
                high_accuracy=True,
                maximum_age=60_000,
                timeout=10_000,
                show_alert=False,
            ),
            dcc.Download(id="telemetry-download"),
            dcc.Interval(
                id="serial-initializer",
                interval=100,
                n_intervals=0,
                max_intervals=1,
            ),
            dcc.Interval(id="refresh-timer", interval=500, n_intervals=0),
            dcc.Interval(
                id="map-refresh-timer",
                interval=MAP_REFRESH_INTERVAL_MS,
                n_intervals=0,
            ),
        ],
        id="app-shell",
        className="app-shell",
        **{"data-theme": "dark"},
    )

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
        Output("reference-path-toggle", "className"),
        Input("reference-path-toggle", "n_clicks"),
        State("reference-path-visible", "data"),
        prevent_initial_call=True,
    )
    def toggle_reference_path(_clicks: int, currently_visible: bool):
        visible = not bool(currently_visible)
        return (
            visible,
            "Hide reference" if visible else "Show reference",
            "button button-active" if visible else "button",
        )

    @app.callback(
        Output("terrain-enabled", "data"),
        Output("terrain-toggle", "children"),
        Output("terrain-toggle", "className"),
        Input("terrain-toggle", "n_clicks"),
        State("terrain-enabled", "data"),
        prevent_initial_call=True,
    )
    def toggle_terrain(_clicks: int, currently_enabled: bool):
        enabled = not bool(currently_enabled)
        return (
            enabled,
            "Flat map" if enabled else "3D terrain",
            "button button-active" if enabled else "button",
        )

    @app.callback(
        Output("trajectory-panel", "className"),
        Output("map-panel", "className"),
        Input("view-mode", "value"),
    )
    def change_view(view_mode: str):
        if view_mode == "map":
            return "visual-panel map-hidden", "visual-panel"
        if view_mode == "split":
            return "visual-panel", "visual-panel"
        return "visual-panel", "visual-panel map-hidden"

    @app.callback(
        Output("serial-control-group", "className"),
        Output("serial-port-select", "disabled"),
        Output("serial-refresh", "disabled"),
        Output("serial-baud-select", "disabled"),
        Output("serial-connect", "disabled"),
        Input("source-mode-revision", "data"),
        Input("playback-revision", "data"),
        Input("serial-revision", "data"),
    )
    def update_serial_control_visibility(
        _source_revision: Optional[dict[str, object]],
        _playback_revision: Optional[dict[str, object]],
        _serial_revision: Optional[int],
    ):
        manager_status = source_manager.status()
        visible = (
            manager_status["live_kind"] == "live"
            and manager_status["mode"] in {"idle", "live"}
        )
        return (
            (
                "control-group serial-control-group"
                if visible
                else "control-group serial-control-group serial-controls-hidden"
            ),
            not visible,
            not visible,
            not visible,
            not visible,
        )

    @app.callback(
        Output("ground-station-popover", "hidden"),
        Input("ground-station-toggle", "n_clicks"),
        Input("ground-station-close", "n_clicks"),
        State("ground-station-popover", "hidden"),
        prevent_initial_call=True,
    )
    def toggle_ground_station_popover(
        _toggle_clicks: int,
        _close_clicks: int,
        currently_hidden: bool,
    ) -> bool:
        if ctx.triggered_id == "ground-station-close":
            return True
        return not bool(currently_hidden)

    @app.callback(
        Output("ground-station-geolocation", "update_now"),
        Input("ground-station-auto", "n_clicks"),
        prevent_initial_call=True,
    )
    def request_ground_station_location(_clicks: int) -> bool:
        return True

    @app.callback(
        Output("ground-station-store", "data"),
        Output("ground-station-latitude", "value"),
        Output("ground-station-longitude", "value"),
        Output("ground-station-altitude", "value"),
        Output("ground-station-status", "children"),
        Input("ground-station-geolocation", "position"),
        Input("ground-station-geolocation", "position_error"),
        Input("ground-station-apply", "n_clicks"),
        State("ground-station-latitude", "value"),
        State("ground-station-longitude", "value"),
        State("ground-station-altitude", "value"),
        State("ground-station-store", "data"),
        prevent_initial_call=True,
    )
    def update_ground_station_location(
        position: Optional[dict[str, object]],
        position_error: Optional[dict[str, object]],
        _apply_clicks: int,
        manual_latitude: object,
        manual_longitude: object,
        manual_altitude: object,
        _current_location: Optional[dict[str, object]],
    ):
        try:
            if ctx.triggered_id == "ground-station-apply":
                location = parse_ground_station_location(
                    manual_latitude,
                    manual_longitude,
                    manual_altitude,
                    source="manual",
                )
                message = "Yer istasyonu konumu kaydedildi"
            elif ctx.triggered_id == "ground-station-geolocation":
                triggered_properties = getattr(ctx, "triggered_prop_ids", {})
                if (
                    "ground-station-geolocation.position_error"
                    in triggered_properties
                    and position_error
                ):
                    return (
                        no_update,
                        no_update,
                        no_update,
                        no_update,
                        "Otomatik konum alınamadı. İzinleri kontrol et veya "
                        "koordinatları elle gir.",
                    )
                if (
                    position
                    and position.get("lat") is not None
                    and position.get("lon") is not None
                ):
                    automatic_altitude = position.get("alt")
                    if automatic_altitude is not None:
                        try:
                            if not math.isfinite(float(automatic_altitude)):
                                automatic_altitude = None
                        except (TypeError, ValueError, OverflowError):
                            automatic_altitude = None
                    automatic_accuracy = position.get("accuracy")
                    if automatic_accuracy is not None:
                        try:
                            if (
                                not math.isfinite(float(automatic_accuracy))
                                or float(automatic_accuracy) < 0.0
                            ):
                                automatic_accuracy = None
                        except (TypeError, ValueError, OverflowError):
                            automatic_accuracy = None
                    location = parse_ground_station_location(
                        position.get("lat"),
                        position.get("lon"),
                        automatic_altitude,
                        automatic_accuracy,
                        source="computer",
                    )
                    accuracy_text = (
                        ""
                        if location.accuracy_m is None
                        else f" · ±{location.accuracy_m:.0f} m"
                    )
                    message = f"Bilgisayar konumu alındı{accuracy_text}"
                elif position_error:
                    return (
                        no_update,
                        no_update,
                        no_update,
                        no_update,
                        "Otomatik konum alınamadı. İzinleri kontrol et veya "
                        "koordinatları elle gir.",
                    )
                else:
                    return (no_update,) * 5
            else:
                return (no_update,) * 5
        except (TypeError, ValueError):
            return (
                no_update,
                no_update,
                no_update,
                no_update,
                "Geçerli bir enlem, boylam ve isteğe bağlı rakım gir.",
            )

        location_data = {
            "latitude_deg": location.latitude_deg,
            "longitude_deg": location.longitude_deg,
            "altitude_m": location.altitude_m,
            "accuracy_m": location.accuracy_m,
            "source": location.source,
        }
        return (
            location_data,
            location.latitude_deg,
            location.longitude_deg,
            location.altitude_m,
            message,
        )

    @app.callback(
        Output("source-mode-revision", "data"),
        Input("source-demo", "n_clicks"),
        Input("source-live", "n_clicks"),
        State("serial-port-select", "value"),
        State("serial-baud-select", "value"),
        State("source-mode-revision", "data"),
        prevent_initial_call=True,
    )
    def select_flight_source(
        _demo_clicks: int,
        _live_clicks: int,
        serial_port: object,
        baud_rate: object,
        revision: Optional[dict[str, object]],
    ) -> dict[str, object]:
        previous_revision = int((revision or {}).get("revision", 0))
        feedback = ""
        try:
            manager_status = source_manager.status()
            if ctx.triggered_id == "source-demo":
                if (
                    manager_status["mode"] == "live"
                    and manager_status["live_kind"] == "demo"
                ):
                    feedback = "Demo flight is already active."
                else:
                    source_manager.activate_demo()
                    live_dvr.reset()
                    feedback = "Demo flight started from a fresh launch · REC OFF"
            elif ctx.triggered_id == "source-live":
                if (
                    manager_status["mode"] in {"live", "idle"}
                    and manager_status["live_kind"] == "live"
                ):
                    feedback = (
                        "Real flight is already connected."
                        if manager_status["mode"] == "live"
                        else "Real flight is ready · select settings, then Connect."
                    )
                else:
                    source_manager.select_serial_mode(serial_port, baud_rate)
                    live_dvr.reset()
                    feedback = (
                        "Real flight is ready · select settings, then Connect "
                        "· REC OFF"
                    )
        except (RuntimeError, TypeError, ValueError) as exc:
            feedback = str(exc)
        return {"revision": previous_revision + 1, "feedback": feedback}

    @app.callback(
        Output("serial-port-select", "options"),
        Output("serial-port-select", "value"),
        Output("serial-baud-select", "options"),
        Output("serial-baud-select", "value"),
        Output("serial-config-message", "children"),
        Output("serial-revision", "data"),
        Input("serial-refresh", "n_clicks"),
        Input("serial-connect", "n_clicks"),
        Input("serial-initializer", "n_intervals"),
        State("serial-port-select", "value"),
        State("serial-baud-select", "value"),
        State("serial-revision", "data"),
        prevent_initial_call=True,
    )
    def manage_serial_connection(
        _refresh_clicks: int,
        _connect_clicks: int,
        _initializer_intervals: int,
        serial_port: object,
        baud_rate: object,
        revision: Optional[int],
    ):
        next_revision = int(revision or 0) + 1
        try:
            if ctx.triggered_id == "serial-initializer":
                manager_status = source_manager.status()
                active_port = manager_status["serial_port"] or DEFAULT_SERIAL_PORT
                active_baud = manager_status["baud_rate"] or DEFAULT_BAUD_RATE
                message = (
                    "Select settings, then Connect"
                    if manager_status["serial_configurable"]
                    else "Unavailable in demo mode"
                )
                return (
                    enumerate_serial_port_options(active_port),
                    active_port,
                    serial_baud_options(active_baud),
                    active_baud,
                    message,
                    next_revision,
                )
            manager_status = source_manager.status()
            if not (
                manager_status["live_kind"] == "live"
                and manager_status["mode"] in {"idle", "live"}
            ):
                raise RuntimeError("Serial controls are available only in Real flight")
            normalized_port, normalized_baud = validate_serial_settings(
                serial_port,
                baud_rate,
            )
            if ctx.triggered_id == "serial-refresh":
                options = enumerate_serial_port_options(normalized_port)
                detected_count = sum(
                    "configured / not detected" not in str(option["label"])
                    for option in options
                )
                return (
                    options,
                    normalized_port,
                    serial_baud_options(normalized_baud),
                    normalized_baud,
                    f"Ports refreshed · {detected_count} detected",
                    next_revision,
                )
            if ctx.triggered_id == "serial-connect":
                normalized_port, normalized_baud = source_manager.configure_serial(
                    normalized_port,
                    normalized_baud,
                )
                live_dvr.reset()
                return (
                    enumerate_serial_port_options(normalized_port),
                    normalized_port,
                    serial_baud_options(normalized_baud),
                    normalized_baud,
                    (
                        f"Opening {normalized_port} at {normalized_baud:,} baud "
                        "· REC OFF"
                    ),
                    next_revision,
                )
        except (RuntimeError, TypeError, ValueError) as exc:
            return (
                no_update,
                no_update,
                no_update,
                no_update,
                str(exc),
                next_revision,
            )
        return (no_update,) * 6

    @app.callback(
        Output("trajectory-name", "children"),
        Output("trajectory-message", "children"),
        Output("trajectory-upload", "contents"),
        Input("trajectory-upload", "contents"),
        Input("clear-trajectory", "n_clicks"),
        State("trajectory-upload", "filename"),
        prevent_initial_call=True,
    )
    def manage_trajectory(contents: Optional[str], _clear_clicks: int, filename: str):
        try:
            if ctx.triggered_id == "clear-trajectory":
                trajectory_store.clear()
                return "Built-in reference", "Custom trajectory cleared", no_update
            if ctx.triggered_id == "trajectory-upload" and contents:
                parsed = parse_trajectory_csv(
                    _decode_dash_upload(contents),
                    filename or "trajectory.csv",
                )
                count = trajectory_store.load(parsed)
                return (
                    parsed.name,
                    f"{count:,} points loaded · {parsed.mode.upper()}",
                    None,
                )
        except (TypeError, ValueError) as exc:
            return "Trajectory load failed", str(exc), None
        if ctx.triggered_id == "trajectory-upload":
            return no_update, no_update, no_update
        name = trajectory_store.snapshot()[1]
        return name, "Load OpenRocket or ENU/LLH CSV", no_update

    @app.callback(
        Output("recording-revision", "data"),
        Input("record-toggle", "n_clicks"),
        State("recording-revision", "data"),
        prevent_initial_call=True,
    )
    def toggle_recording(n_clicks: int, revision: Optional[int]):
        if n_clicks and ctx.triggered_id == "record-toggle":
            source_manager.toggle_recording()
        return int(revision or 0) + 1

    @app.callback(
        Output("telemetry-download", "data"),
        Input("export-recording", "n_clicks"),
        prevent_initial_call=True,
    )
    def export_recording(_clicks: int):
        points = store.recording_snapshot()
        if not points:
            return no_update
        filename = f"gnss_recording_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        return dcc.send_string(telemetry_points_to_csv(points), filename)

    @app.callback(
        Output("playback-revision", "data"),
        Output("replay-upload", "contents"),
        Input("replay-upload", "contents"),
        Input("replay-play", "n_clicks"),
        Input("replay-restart", "n_clicks"),
        Input("return-live", "n_clicks"),
        Input("replay-speed", "value"),
        State("replay-upload", "filename"),
        State("serial-port-select", "value"),
        State("serial-baud-select", "value"),
        State("playback-revision", "data"),
        prevent_initial_call=True,
    )
    def manage_replay(
        replay_contents: Optional[str],
        _play_clicks: int,
        _restart_clicks: int,
        _live_clicks: int,
        speed: float,
        filename: str,
        serial_port: object,
        baud_rate: object,
        revision: Optional[dict[str, object]],
    ):
        feedback = ""
        clear_upload = no_update
        try:
            trigger = ctx.triggered_id
            if trigger == "replay-upload" and replay_contents:
                samples = parse_telemetry_csv(_decode_dash_upload(replay_contents))
                source_manager.load_replay(samples, filename or "telemetry.csv")
                live_dvr.reset()
                if speed is not None:
                    source_manager.set_replay_speed(float(speed))
                feedback = f"{len(samples):,} telemetry samples loaded"
                if not bool(source_manager.status()["launch_known"]):
                    feedback += " · partial descent; launch/ascent is missing"
                clear_upload = None
            elif trigger == "replay-upload":
                return no_update, no_update
            elif trigger == "replay-play":
                if source_manager.status()["mode"] == "replay":
                    source_manager.toggle_replay()
                elif source_manager.status()["mode"] == "live":
                    live_points, _ = store.snapshot()
                    live_dvr.toggle(live_points)
            elif trigger == "replay-restart":
                if source_manager.status()["mode"] == "replay":
                    source_manager.seek_replay(0.0)
                elif source_manager.status()["mode"] == "live":
                    live_points, _ = store.snapshot()
                    if live_points:
                        live_dvr.seek(live_points[0].time_s, live_points)
            elif trigger == "return-live":
                manager_status = source_manager.status()
                if manager_status["mode"] == "live":
                    live_points, _ = store.snapshot()
                    dvr_status = live_dvr.status(live_points)
                    if bool(dvr_status["at_live_edge"]):
                        feedback = "Already at the live edge"
                    else:
                        live_dvr.go_live(live_points)
                        feedback = "Jumped to LIVE · receiver and recording were not interrupted"
                elif manager_status["mode"] == "replay":
                    if manager_status["live_kind"] == "live":
                        connected_port, connected_baud = (
                            source_manager.configure_serial(serial_port, baud_rate)
                        )
                        live_dvr.reset()
                        feedback = (
                            f"Live source reconnected on {connected_port} at "
                            f"{connected_baud:,} baud · recording OFF"
                        )
                    elif manager_status["live_kind"] == "demo":
                        source_manager.activate_demo()
                        live_dvr.reset()
                        feedback = "Demo source resumed · recording OFF"
                    else:
                        feedback = "Choose Demo flight or Real flight before returning."
                else:
                    feedback = "No replay is active · recording OFF"
            elif trigger == "replay-speed" and speed is not None:
                if source_manager.status()["mode"] == "replay":
                    source_manager.set_replay_speed(float(speed))
                elif source_manager.status()["mode"] == "live":
                    live_dvr.set_speed(float(speed))
        except (RuntimeError, TypeError, ValueError) as exc:
            feedback = str(exc)
            if ctx.triggered_id == "replay-upload":
                clear_upload = None
        previous_revision = int((revision or {}).get("revision", 0))
        return (
            {"revision": previous_revision + 1, "feedback": feedback},
            clear_upload,
        )

    @app.callback(
        Input("replay-slider", "value"),
        State("rendered-playhead", "data"),
        prevent_initial_call=True,
    )
    def seek_replay_from_slider(
        slider_value: Optional[float],
        rendered_playhead: Optional[float],
    ) -> None:
        if slider_value is None:
            return
        if rendered_playhead is not None and math.isclose(
            float(slider_value),
            float(rendered_playhead),
            abs_tol=1e-6,
        ):
            return
        try:
            if source_manager.status()["mode"] == "replay":
                source_manager.seek_replay(float(slider_value))
            elif source_manager.status()["mode"] == "live":
                live_points, _ = store.snapshot()
                live_dvr.seek(float(slider_value), live_points)
        except (TypeError, ValueError) as exc:
            store.set_status("error", f"Timeline seek failed: {exc}")

    @app.callback(
        Output("app-shell", "data-theme"),
        Output("theme-toggle", "children"),
        Output("trajectory-graph", "figure"),
        Output("profile-graph", "figure"),
        Output("source-status", "children"),
        Output("source-status-chip", "data-status"),
        Output("source-mode", "children"),
        Output("source-label", "children"),
        Output("source-message", "children"),
        Output("source-choice-message", "children"),
        Output("source-demo", "className"),
        Output("source-live", "className"),
        Output("latitude-value", "children"),
        Output("longitude-value", "children"),
        Output("altitude-value", "children"),
        Output("up-value", "children"),
        Output("ground-speed-value", "children"),
        Output("vertical-speed-value", "children"),
        Output("range-value", "children"),
        Output("max-up-value", "children"),
        Output("flight-time-value", "children"),
        Output("distance-value", "children"),
        Output("phase-value", "children"),
        Output("accepted-value", "children"),
        Output("rejected-value", "children"),
        Output("missing-value", "children"),
        Output("record-indicator", "children"),
        Output("record-toggle", "children"),
        Output("record-toggle", "className"),
        Output("record-toggle", "disabled"),
        Output("return-live", "children"),
        Output("return-live", "disabled"),
        Output("replay-play", "children"),
        Output("replay-play", "className"),
        Output("replay-play", "disabled"),
        Output("replay-restart", "children"),
        Output("replay-restart", "disabled"),
        Output("replay-speed", "disabled"),
        Output("replay-file-name", "children"),
        Output("replay-time", "children"),
        Output("replay-time", "className"),
        Output("replay-slider", "value"),
        Output("rendered-playhead", "data"),
        Output("replay-slider", "min"),
        Output("replay-slider", "max"),
        Output("replay-slider", "marks"),
        Output("replay-slider", "disabled"),
        Output("playback-message", "children"),
        Output("map-stream-data", "data"),
        Output("map-live-badge", "children"),
        Input("refresh-timer", "n_intervals"),
        Input("reset-track", "n_clicks"),
        Input("theme-mode", "data"),
        Input("reference-path-visible", "data"),
        Input("playback-revision", "data"),
        Input("recording-revision", "data"),
        Input("serial-revision", "data"),
        Input("source-mode-revision", "data"),
        Input("view-mode", "value"),
        Input("reset-3d-view", "n_clicks"),
        State("rendered-playhead", "data"),
        State("replay-slider", "value"),
    )
    def update_dashboard(
        _interval: int,
        _reset_clicks: int,
        theme: str,
        show_reference: bool,
        playback_revision: Optional[dict[str, object]],
        _recording_revision: Optional[int],
        _serial_revision: Optional[int],
        source_mode_revision: Optional[dict[str, object]],
        view_mode: str,
        camera_revision: int,
        rendered_playhead: Optional[float],
        slider_playhead: Optional[float],
    ):
        manager_status = source_manager.status()
        if ctx.triggered_id == "reset-track":
            if manager_status["mode"] == "replay":
                source_manager.seek_replay(0.0)
            else:
                store.clear_track(reset_counters=True)
                live_dvr.reset()

        source_manager.tick()
        manager_status = source_manager.status()
        all_points, source = store.snapshot()
        if manager_status["mode"] == "live":
            live_dvr.tick(all_points)
            dvr_status = live_dvr.status(all_points)
            points = live_dvr.visible_points(all_points)
        else:
            dvr_status = {
                "at_live_edge": True,
                "playing": True,
                "playhead_s": 0.0,
                "speed": 1.0,
                "minimum_s": 0.0,
                "live_edge_s": 0.0,
                "behind_s": 0.0,
                "count": 0,
            }
            points = all_points

        trajectory_points, trajectory_name = trajectory_store.snapshot()
        partial_replay = (
            manager_status["mode"] == "replay"
            and not bool(manager_status["launch_known"])
        )
        custom_reference_loaded = bool(trajectory_points)
        reference_available = not partial_replay or custom_reference_loaded
        reference_notice = (
            PARTIAL_REPLAY_PLOT_NOTICE
            if partial_replay and not custom_reference_loaded
            else None
        )
        visible_trajectory_points = trajectory_points if show_reference else []
        launch_label = "Replay start" if partial_replay else "Launch"
        theme = theme if theme in FIGURE_THEMES else "dark"

        status = str(source["status"])
        message = str(source["message"])
        if partial_replay and PARTIAL_REPLAY_NOTICE not in message:
            message = f"{message} · {PARTIAL_REPLAY_NOTICE}"
        last_receive_epoch = source["last_receive_epoch"]
        packet_age_s: Optional[float] = None
        if last_receive_epoch is not None:
            packet_age_s = max(0.0, time.time() - float(last_receive_epoch))
        if (
            manager_status["mode"] == "live"
            and status in {"connected", "demo"}
            and packet_age_s is not None
            and packet_age_s > stale_timeout_s
        ):
            status = "stale"
            message = f"No telemetry for {packet_age_s:.1f} s · last values are frozen"

        if not points:
            metric_values = ("--",) * 10
            phase = "STANDBY"
        else:
            current = points[-1]
            ground_speed, vertical_speed = calculate_speed(points)
            ground_range = math.hypot(current.east_m, current.north_m)
            if (
                manager_status["mode"] == "live"
                and not bool(dvr_status["at_live_edge"])
            ):
                displayed_max_up_m = max(
                    0.0,
                    max((point.up_m for point in points), default=0.0),
                )
                displayed_distance_m = sum(
                    math.sqrt(
                        (right.east_m - left.east_m) ** 2
                        + (right.north_m - left.north_m) ** 2
                        + (right.up_m - left.up_m) ** 2
                    )
                    for left, right in zip(points, points[1:])
                )
            else:
                displayed_max_up_m = float(source["max_up_m"])
                displayed_distance_m = float(source["distance_travelled_m"])
            phase = infer_flight_phase(points, vertical_speed)
            metric_values = (
                f"{current.latitude_deg:.7f}",
                f"{current.longitude_deg:.7f}",
                f"{current.altitude_m:.1f}",
                f"{current.up_m:.1f}",
                f"{ground_speed:.1f}",
                f"{vertical_speed:+.1f}",
                f"{ground_range:.1f}",
                f"{displayed_max_up_m:.1f}",
                f"{current.time_s:.1f}",
                f"{displayed_distance_m:.1f}",
            )

        chip_status = "replay" if status == "replay_complete" else status
        status_label = "COMPLETE" if status == "replay_complete" else status.upper()
        recorded_count = int(source["recorded_count"])
        if source["recording"]:
            recording_label = f"REC ON · {recorded_count:,}"
        elif recorded_count:
            recording_label = f"REC OFF · {recorded_count:,} saved"
        else:
            recording_label = "REC OFF"
        live_mode = manager_status["mode"] == "live"
        demo_source = manager_status["live_kind"] == "demo"
        real_source = manager_status["live_kind"] == "live"
        recording = bool(source["recording"])
        age_suffix = (
            ""
            if packet_age_s is None or manager_status["mode"] == "replay"
            else f" · packet age {packet_age_s:.1f} s"
        )
        replay_loaded = bool(manager_status["loaded"]) and manager_status["mode"] == "replay"
        if replay_loaded:
            timeline_available = float(manager_status["duration_s"]) > 0.0
            timeline_playing = bool(manager_status["playing"])
            timeline_playhead = float(manager_status["playhead_s"])
            timeline_minimum = 0.0
            timeline_edge = float(manager_status["duration_s"])
            timeline_at_live_edge = False
            timeline_speed = float(manager_status["speed"])
            timeline_name = str(manager_status["name"])
            timeline_time_label = (
                f"{_format_clock(timeline_playhead)} / "
                f"{_format_clock(timeline_edge)}"
            )
            timeline_time_class = "replay-time"
        elif live_mode:
            timeline_available = len(all_points) >= 2
            timeline_playing = bool(dvr_status["playing"])
            timeline_playhead = float(dvr_status["playhead_s"])
            timeline_minimum = float(dvr_status["minimum_s"])
            timeline_edge = float(dvr_status["live_edge_s"])
            timeline_at_live_edge = bool(dvr_status["at_live_edge"])
            timeline_speed = float(dvr_status["speed"])
            timeline_name = f"Live DVR buffer · {len(all_points):,} samples"
            timeline_time_label = (
                "LIVE"
                if timeline_at_live_edge
                else f"-{_format_clock(float(dvr_status['behind_s']))} / LIVE"
            )
            timeline_time_class = (
                "replay-time timeline-live"
                if timeline_at_live_edge
                else "replay-time timeline-behind"
            )
        else:
            timeline_available = False
            timeline_playing = False
            timeline_playhead = 0.0
            timeline_minimum = 0.0
            timeline_edge = 0.0
            timeline_at_live_edge = False
            timeline_speed = 1.0
            timeline_name = "No live source or replay"
            timeline_time_label = "00:00.0 / 00:00.0"
            timeline_time_class = "replay-time"

        publish_playhead = (
            rendered_playhead is None
            or slider_playhead is None
            or not math.isclose(
                timeline_playhead,
                float(rendered_playhead),
                abs_tol=1e-6,
            )
            or not math.isclose(
                timeline_playhead,
                float(slider_playhead),
                abs_tol=1e-6,
            )
        )
        timeline_slider_value = timeline_playhead if publish_playhead else no_update
        rendered_playhead_value = timeline_playhead if publish_playhead else no_update
        timeline_marks = {timeline_minimum: _format_clock(timeline_minimum)}
        if timeline_edge > timeline_minimum:
            timeline_marks[timeline_edge] = (
                "LIVE" if live_mode and not replay_loaded else _format_clock(timeline_edge)
            )
        action_feedback = str((playback_revision or {}).get("feedback", ""))
        if ctx.triggered_id != "playback-revision" or not action_feedback:
            if replay_loaded:
                replay_state = "Playing" if timeline_playing else "Paused"
                action_feedback = (
                    f"{replay_state} · sample {int(manager_status['index']):,}/"
                    f"{int(manager_status['count']):,} · "
                    f"{timeline_speed:g}×"
                )
            elif live_mode and timeline_available:
                if timeline_at_live_edge:
                    action_feedback = (
                        f"LIVE · {len(all_points):,} samples retained · "
                        "drag the timeline to rewind"
                    )
                else:
                    action_feedback = (
                        f"{float(dvr_status['behind_s']):.1f} s behind LIVE · "
                        "receiver and recording continue"
                    )
            else:
                action_feedback = "Start Demo/Real flight or load a recorded CSV."

        source_choice_feedback = str(
            (source_mode_revision or {}).get("feedback", "")
        )
        if ctx.triggered_id == "source-mode-revision" and source_choice_feedback:
            source_choice_message = source_choice_feedback
        elif manager_status["mode"] == "replay":
            source_choice_message = (
                "Replay is active. Choose Demo flight or Real flight to leave playback."
            )
        elif live_mode and not timeline_at_live_edge:
            source_choice_message = (
                f"DVR view · {float(dvr_status['behind_s']):.1f} s behind LIVE · "
                "telemetry continues in the background."
            )
        elif live_mode and demo_source:
            source_choice_message = (
                "Demo flight is active · synthetic telemetry · REC starts only on request."
            )
        elif live_mode and real_source:
            source_choice_message = (
                "Real flight is active · live serial telemetry · REC starts only on request."
            )
        elif real_source:
            source_choice_message = (
                "Real flight is ready · choose a COM port and baud rate, then Connect."
            )
        else:
            source_choice_message = (
                "Choose Demo flight for a simulated mission or Real flight for COM telemetry."
            )

        if live_mode and not timeline_at_live_edge:
            source_mode_label = "DVR"
        elif live_mode and demo_source:
            source_mode_label = "SIMULATION"
        elif live_mode and real_source:
            source_mode_label = "LIVE"
        elif manager_status["mode"] == "replay":
            source_mode_label = "REPLAY"
        elif real_source:
            source_mode_label = "SERIAL READY"
        else:
            source_mode_label = "CHOOSE MODE"

        trajectory_figure = (
            no_update
            if view_mode == "map"
            else create_trajectory_figure(
                points,
                theme,
                bool(show_reference),
                visible_trajectory_points,
                trajectory_name,
                source["launch_point"],
                int(camera_revision or 0),
                launch_label,
                reference_available,
                reference_notice,
            )
        )
        if points:
            map_points = _downsample_for_display(points)
            map_current = map_points[-1]
            map_stream_data = {
                "revision": (
                    f"{manager_status['mode']}-{len(points)}-"
                    f"{map_current.time_s:.6f}"
                ),
                "latitudes": [point.latitude_deg for point in map_points],
                "longitudes": _unwrap_longitudes(
                    [point.longitude_deg for point in map_points]
                ),
                "relative_up": [point.up_m for point in map_points],
                "track_customdata": [
                    [
                        point.altitude_m,
                        point.up_m,
                        point.time_s,
                        point.longitude_deg,
                    ]
                    for point in map_points
                ],
                "current": {
                    "latitude_deg": map_current.latitude_deg,
                    "longitude_deg": map_current.longitude_deg,
                    "altitude_m": map_current.altitude_m,
                    "up_m": map_current.up_m,
                    "time_s": map_current.time_s,
                    "label": f"🚀 {map_current.up_m:+,.0f} m",
                },
            }
            map_live_badge = (
                f"🚀 ROCKET  {map_current.up_m:+,.0f} m  ·  "
                f"{map_current.time_s:.1f} s"
            )
        else:
            map_stream_data = {
                "revision": f"{manager_status['mode']}-empty",
                "latitudes": [],
            }
            map_live_badge = ""

        return (
            theme,
            "Dark mode" if theme == "light" else "Light mode",
            trajectory_figure,
            create_profile_figure(points, theme, visible_trajectory_points),
            status_label,
            chip_status,
            source_mode_label,
            str(manager_status["source_label"]),
            message + age_suffix,
            source_choice_message,
            (
                "source-choice-button source-choice-active"
                if demo_source
                else "source-choice-button"
            ),
            (
                "source-choice-button source-choice-active"
                if real_source
                else "source-choice-button"
            ),
            *metric_values,
            phase,
            f"{int(source['accepted']):,} accepted",
            f"{int(source['rejected']):,} rejected",
            (
                f"{int(source['missing']):,} missing · "
                f"{int(source['duplicates']):,} duplicates · "
                f"{int(source['out_of_order']):,} out of order"
            ),
            recording_label,
            "Stop recording" if recording else "Start recording",
            "button button-danger" if recording else "button button-primary",
            not live_mode,
            (
                "LIVE"
                if live_mode and timeline_at_live_edge
                else "Go LIVE"
                if live_mode
                else "Return to demo"
                if manager_status["mode"] == "replay" and demo_source
                else "Return to live"
                if manager_status["mode"] == "replay" and real_source
                else "Real flight ready"
                if real_source
                else "Choose mode"
            ),
            (
                timeline_at_live_edge
                if live_mode
                else manager_status["mode"] != "replay"
            ),
            "Pause" if timeline_playing else "Play",
            (
                "button button-danger"
                if timeline_playing
                else "button button-primary"
            ),
            not timeline_available,
            "Buffer start" if live_mode else "Restart",
            not timeline_available,
            not timeline_available,
            timeline_name,
            timeline_time_label,
            timeline_time_class,
            timeline_slider_value,
            rendered_playhead_value,
            timeline_minimum,
            max(timeline_minimum + 0.1, timeline_edge),
            timeline_marks,
            not timeline_available,
            action_feedback,
            map_stream_data,
            map_live_badge,
        )

    @app.callback(
        Output("map-graph", "figure"),
        Input("map-refresh-timer", "n_intervals"),
        Input("theme-mode", "data"),
        Input("reference-path-visible", "data"),
        Input("terrain-enabled", "data"),
        Input("playback-revision", "data"),
        Input("serial-revision", "data"),
        Input("source-mode-revision", "data"),
        Input("view-mode", "value"),
        Input("ground-station-store", "data"),
        Input("reset-track", "n_clicks"),
    )
    def update_map(
        _interval: int,
        theme: str,
        show_reference: bool,
        terrain_enabled: bool,
        _playback_revision: Optional[dict[str, object]],
        _serial_revision: Optional[int],
        _source_mode_revision: Optional[dict[str, object]],
        view_mode: str,
        ground_station_data: Optional[dict[str, object]],
        _reset_clicks: int,
    ):
        """Render MapLibre independently so its slower repaint cannot lag telemetry."""
        if view_mode == "3d":
            return no_update

        manager_status = source_manager.status()
        all_points, source = store.snapshot()
        points = (
            live_dvr.visible_points(all_points)
            if manager_status["mode"] == "live"
            else all_points
        )

        ground_station: Optional[GroundStationLocation] = None
        if isinstance(ground_station_data, dict):
            try:
                ground_station = parse_ground_station_location(
                    ground_station_data.get("latitude_deg"),
                    ground_station_data.get("longitude_deg"),
                    ground_station_data.get("altitude_m"),
                    ground_station_data.get("accuracy_m"),
                    ground_station_data.get("source", "manual"),
                )
            except (TypeError, ValueError):
                ground_station = None

        trajectory_points, trajectory_name = trajectory_store.snapshot()
        partial_replay = (
            manager_status["mode"] == "replay"
            and not bool(manager_status["launch_known"])
        )
        reference_available = not partial_replay or bool(trajectory_points)
        visible_trajectory_points = trajectory_points if show_reference else []
        launch_label = "Replay start" if partial_replay else "Launch"
        selected_theme = theme if theme in FIGURE_THEMES else "dark"

        figure = create_map_figure(
            points,
            selected_theme,
            visible_trajectory_points,
            source["launch_point"],
            bool(terrain_enabled),
            launch_label,
            bool(show_reference and reference_available),
            trajectory_name,
            ground_station,
            map_empty_state_message(
                manager_status["mode"],
                manager_status["live_kind"],
                max(0, int(_interval or 0))
                * MAP_REFRESH_INTERVAL_MS
                / 1_000.0,
            ),
        )
        return figure

    app.clientside_callback(
        """
        function(payload) {
            if (!payload || !payload.current) {
                return window.dash_clientside.no_update;
            }
            if (!window.Plotly) {
                return window.dash_clientside.no_update;
            }
            const graphWrapper = document.getElementById("map-graph");
            const graph = graphWrapper
                ? graphWrapper.querySelector(".js-plotly-plot")
                : null;
            if (!graph || !Array.isArray(graph.data)) {
                return window.dash_clientside.no_update;
            }
            const trackIndex = graph.data.findIndex(
                (trace) => trace && trace.name === "Ground track"
            );
            const currentIndex = graph.data.findIndex(
                (trace) => trace && trace.name === "Current"
            );
            if (trackIndex < 0 || currentIndex < 0) {
                return window.dash_clientside.no_update;
            }
            try {
                window.Plotly.restyle(
                    graph,
                    {
                        lat: [payload.latitudes],
                        lon: [payload.longitudes],
                        customdata: [payload.track_customdata],
                        "marker.color": [payload.relative_up]
                    },
                    [trackIndex]
                );
                const current = payload.current;
                window.Plotly.restyle(
                    graph,
                    {
                        lat: [[current.latitude_deg]],
                        lon: [[current.longitude_deg]],
                        text: [[current.label]],
                        customdata: [[[
                            current.longitude_deg,
                            current.altitude_m,
                            current.up_m,
                            current.time_s
                        ]]]
                    },
                    [currentIndex]
                );
            } catch (error) {
                return window.dash_clientside.no_update;
            }
            return payload.revision;
        }
        """,
        Output("map-stream-applied", "data"),
        Input("map-stream-data", "data"),
        prevent_initial_call=True,
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
        "--max-points",
        type=int,
        default=20_000,
        help="Maximum live/recording points retained in memory (default: 20000)",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=DEFAULT_MAX_PLAUSIBLE_SPEED_MPS,
        help="Reject fixes faster than this 3D speed in m/s; 0 disables filtering",
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=DEFAULT_STALE_TIMEOUT_S,
        help="Seconds without telemetry before the source is marked stale",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Advanced: start the synthetic flight immediately",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Do not open the dashboard in the default browser",
    )
    args = parser.parse_args()
    if not 1 <= args.http_port <= 65_535:
        parser.error("--http-port must be between 1 and 65535")
    try:
        args.serial_port, args.baud = validate_serial_settings(
            args.serial_port,
            args.baud,
        )
    except ValueError as exc:
        parser.error(str(exc))
    if not 100 <= args.max_points <= MAX_POINT_CAPACITY:
        parser.error(
            f"--max-points must be between 100 and {MAX_POINT_CAPACITY:,}"
        )
    if not math.isfinite(args.max_speed) or args.max_speed < 0.0:
        parser.error("--max-speed must be finite and cannot be negative")
    if not math.isfinite(args.stale_timeout) or args.stale_timeout <= 0.0:
        parser.error("--stale-timeout must be finite and positive")
    return args


def main() -> None:
    args = parse_arguments()
    load_dashboard_dependencies()
    store = TelemetryStore(
        max_points=args.max_points,
        max_plausible_speed_mps=args.max_speed,
    )

    if args.demo:
        live_source_factory = create_demo_source_factory(store)
        source_label = "DEMO source / synthetic 78-byte HYI packet model"
        live_kind = "demo"
        initial_mode = "live"
    else:
        live_source_factory = lambda: SerialReceiver(
            store, args.serial_port, args.baud
        )
        source_label = "Choose Demo flight or Real flight"
        live_kind = "idle"
        initial_mode = "idle"

    source_manager = SourceManager(
        store,
        live_source_factory,
        source_label,
        live_kind=live_kind,
        serial_port=args.serial_port,
        baud_rate=args.baud,
        initial_mode=initial_mode,
    )
    trajectory_store = TrajectoryStore()
    if args.demo:
        source_manager.start_live()
    else:
        store.set_status(
            "waiting",
            "Choose Demo flight or Real flight to begin",
        )
    atexit.register(source_manager.stop)

    app = build_dash_app(
        store,
        source_manager,
        trajectory_store,
        stale_timeout_s=args.stale_timeout,
    )
    dashboard_url = f"http://127.0.0.1:{args.http_port}"
    print(f"Dashboard: {dashboard_url}")

    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(dashboard_url)).start()

    try:
        app.run(host="127.0.0.1", port=args.http_port, debug=False)
    finally:
        source_manager.stop()


if __name__ == "__main__":
    main()
