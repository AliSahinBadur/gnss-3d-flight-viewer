"""CSV import/export helpers for recorded GNSS telemetry.

The module deliberately depends only on the Python standard library so that
recordings can be inspected and converted without installing the dashboard
dependencies.
"""

from __future__ import annotations

import csv
import io
import math
import re
import unicodedata
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Iterable


@dataclass(frozen=True)
class ReplaySample:
    """A position sample in the time domain used by the replay controller."""

    time_s: float
    packet_counter: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    epoch_s: float | None = None


_TIME_ALIASES = {
    "time",
    "time_s",
    "elapsed",
    "elapsed_s",
    "elapsed_time",
    "flight_time",
}
_TIMESTAMP_ALIASES = {"timestamp"}
_EPOCH_ALIASES = {"epoch", "epoch_s", "unix_time", "unix_timestamp"}
_LATITUDE_ALIASES = {
    "latitude",
    "latitude_deg",
    "lat",
    "lat_deg",
    "gps_latitude",
    "enlem",
}
_LONGITUDE_ALIASES = {
    "longitude",
    "longitude_deg",
    "lon",
    "lon_deg",
    "lng",
    "gps_longitude",
    "boylam",
}
_ALTITUDE_ALIASES = {
    "altitude",
    "altitude_m",
    "alt",
    "alt_m",
    "gnss_altitude",
    "irtifa",
}
_COUNTER_ALIASES = {"counter", "packet_counter"}

_HEADER_KINDS = {
    **{alias: "time" for alias in _TIME_ALIASES},
    **{alias: "timestamp" for alias in _TIMESTAMP_ALIASES},
    **{alias: "epoch" for alias in _EPOCH_ALIASES},
    **{alias: "latitude" for alias in _LATITUDE_ALIASES},
    **{alias: "longitude" for alias in _LONGITUDE_ALIASES},
    **{alias: "altitude" for alias in _ALTITUDE_ALIASES},
    **{alias: "counter" for alias in _COUNTER_ALIASES},
}
_REQUIRED_KINDS = {"latitude", "longitude", "altitude"}
_DELIMITERS = (",", ";", "\t")
_UNIX_EPOCH_HEURISTIC_S = 100_000_000.0
_MIN_PLAUSIBLE_ALTITUDE_M = -5_000.0
_MAX_PLAUSIBLE_ALTITUDE_M = 1_000_000.0
MAX_CSV_BYTES = 16 * 1024 * 1024
MAX_REPLAY_SAMPLES = 250_000


def _decode_csv(data: str | bytes) -> str:
    if isinstance(data, str):
        if len(data) > MAX_CSV_BYTES:
            raise ValueError("Telemetry CSV exceeds the 16 MiB size limit")
        return data.lstrip("\ufeff")
    if not isinstance(data, bytes):
        raise TypeError("Telemetry CSV data must be str or bytes")
    if len(data) > MAX_CSV_BYTES:
        raise ValueError("Telemetry CSV exceeds the 16 MiB size limit")

    for encoding in ("utf-8-sig", "cp1254"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise ValueError("Telemetry CSV is neither valid UTF-8 nor Windows-1254")


def _is_comment(line: str) -> bool:
    stripped = line.lstrip()
    return stripped.startswith("#") or stripped.startswith("//")


def _prepare_lines(text: str) -> tuple[list[str], str | None]:
    lines: list[str] = []
    declared_delimiter: str | None = None
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or _is_comment(line):
            continue
        if not lines and stripped.casefold().startswith("sep="):
            candidate = stripped[4:]
            if candidate in _DELIMITERS:
                declared_delimiter = candidate
                continue
        lines.append(line)
    return lines, declared_delimiter


def _normalise_header(value: str) -> str:
    value = value.strip().lstrip("\ufeff")
    value = re.sub(r"\([^)]*\)|\[[^]]*]", "", value)
    value = unicodedata.normalize("NFKD", value.casefold())
    value = "".join(character for character in value if not unicodedata.combining(character))
    return re.sub(r"[^a-z0-9]+", "_", value).strip("_")


def _row_header_map(row: list[str]) -> dict[str, int]:
    mapping: dict[str, int] = {}
    for index, cell in enumerate(row):
        kind = _HEADER_KINDS.get(_normalise_header(cell))
        if kind is not None and kind not in mapping:
            mapping[kind] = index
    return mapping


def _find_format(
    lines: list[str],
    declared_delimiter: str | None,
) -> tuple[str, int, dict[str, int]]:
    delimiters = (declared_delimiter,) if declared_delimiter else _DELIMITERS
    best: tuple[tuple[int, int, int], str, int, dict[str, int]] | None = None

    for delimiter in delimiters:
        try:
            rows = csv.reader(lines, delimiter=delimiter)
        except csv.Error:
            continue
        try:
            for row_index, row in enumerate(rows):
                if row_index >= 100:
                    break
                mapping = _row_header_map(row)
                kinds = set(mapping)
                score = (
                    int(_REQUIRED_KINDS <= kinds),
                    len(kinds),
                    len(row),
                )
                if best is None or score > best[0]:
                    best = (score, delimiter, row_index, mapping)
        except csv.Error:
            continue

    if best is None or not _REQUIRED_KINDS <= set(best[3]):
        raise ValueError(
            "Telemetry CSV header must contain latitude, longitude, and altitude columns"
        )
    return best[1], best[2], best[3]


def _parse_number(value: object) -> float:
    text = str(value).strip().replace("\u00a0", "").replace(" ", "")
    if not text:
        raise ValueError("missing number")

    # Accept decimal-comma exports commonly produced by Turkish spreadsheet
    # installations, while retaining support for conventional thousands marks.
    if "," in text and "." in text:
        if text.rfind(",") > text.rfind("."):
            text = text.replace(".", "").replace(",", ".")
        else:
            text = text.replace(",", "")
    elif "," in text:
        text = text.replace(",", ".")

    number = float(text)
    if not math.isfinite(number):
        raise ValueError("number is not finite")
    return number


def _parse_counter(value: object) -> int:
    number = _parse_number(value)
    counter = int(number)
    if number != counter or counter < 0:
        raise ValueError("packet counter must be a non-negative integer")
    return counter


def _parse_datetime_epoch(value: object) -> tuple[float, bool]:
    text = str(value).strip()
    try:
        return _parse_number(text), False
    except (TypeError, ValueError):
        pass

    iso_text = text[:-1] + "+00:00" if text.endswith(("Z", "z")) else text
    parsed = datetime.fromisoformat(iso_text)
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    epoch_s = parsed.timestamp()
    if not math.isfinite(epoch_s):
        raise ValueError("timestamp is not finite")
    return epoch_s, True


def _cell(row: list[str], mapping: dict[str, int], kind: str) -> str:
    index = mapping[kind]
    if index >= len(row):
        raise ValueError(f"missing {kind}")
    return row[index]


def parse_telemetry_csv(
    data: str | bytes,
    default_interval_s: float = 0.1,
) -> list[ReplaySample]:
    """Parse a telemetry CSV, skipping malformed or out-of-range data rows.

    Files may be UTF-8 (with or without a BOM) or Windows-1254, and may use a
    comma, semicolon, or tab delimiter.  When no time column exists, accepted
    samples are spaced by ``default_interval_s``.  A ``timestamp`` containing
    Unix epoch seconds (or an ISO-8601 date) is converted to elapsed replay time
    relative to the first accepted sample while the original epoch is retained.
    """

    try:
        interval = float(default_interval_s)
    except (TypeError, ValueError) as exc:
        raise ValueError("default_interval_s must be a positive finite number") from exc
    if not math.isfinite(interval) or interval <= 0.0:
        raise ValueError("default_interval_s must be a positive finite number")

    text = _decode_csv(data)
    lines, declared_delimiter = _prepare_lines(text)
    if not lines:
        raise ValueError("Telemetry CSV contains no header or data rows")

    delimiter, header_index, mapping = _find_format(lines, declared_delimiter)
    samples: list[ReplaySample] = []
    first_epoch_s: float | None = None
    timestamp_is_epoch: bool | None = None
    has_time = "time" in mapping
    has_timestamp = "timestamp" in mapping
    has_epoch = "epoch" in mapping

    try:
        rows = csv.reader(lines, delimiter=delimiter)
        for row_index, row in enumerate(rows):
            if row_index <= header_index:
                continue
            if not row or not any(cell.strip() for cell in row):
                continue
            try:
                latitude = _parse_number(_cell(row, mapping, "latitude"))
                longitude = _parse_number(_cell(row, mapping, "longitude"))
                altitude = _parse_number(_cell(row, mapping, "altitude"))
                if not -90.0 <= latitude <= 90.0:
                    raise ValueError("latitude is outside [-90, 90]")
                if not -180.0 <= longitude <= 180.0:
                    raise ValueError("longitude is outside [-180, 180]")
                if not (
                    _MIN_PLAUSIBLE_ALTITUDE_M
                    <= altitude
                    <= _MAX_PLAUSIBLE_ALTITUDE_M
                ):
                    raise ValueError("altitude is outside the plausible range")
                if abs(latitude) < 1e-9 and abs(longitude) < 1e-9:
                    raise ValueError("GNSS fix is unavailable")

                if "counter" in mapping:
                    packet_counter = _parse_counter(_cell(row, mapping, "counter"))
                else:
                    packet_counter = len(samples)

                epoch_s: float | None = None
                if has_time:
                    time_s = _parse_number(_cell(row, mapping, "time"))
                    optional_epoch_kind = (
                        "epoch"
                        if has_epoch
                        else ("timestamp" if has_timestamp else None)
                    )
                    if optional_epoch_kind is not None:
                        raw_epoch = _cell(row, mapping, optional_epoch_kind).strip()
                        if raw_epoch:
                            epoch_s, _ = _parse_datetime_epoch(raw_epoch)
                elif has_epoch:
                    epoch_s, _ = _parse_datetime_epoch(_cell(row, mapping, "epoch"))
                    if first_epoch_s is None:
                        first_epoch_s = epoch_s
                    time_s = epoch_s - first_epoch_s
                elif has_timestamp:
                    timestamp_s, explicitly_epoch = _parse_datetime_epoch(
                        _cell(row, mapping, "timestamp")
                    )
                    if timestamp_is_epoch is None:
                        timestamp_is_epoch = (
                            explicitly_epoch
                            or abs(timestamp_s) >= _UNIX_EPOCH_HEURISTIC_S
                        )
                    if timestamp_is_epoch:
                        epoch_s = timestamp_s
                        if first_epoch_s is None:
                            first_epoch_s = epoch_s
                        time_s = epoch_s - first_epoch_s
                    else:
                        time_s = timestamp_s
                else:
                    time_s = len(samples) * interval

                sample = ReplaySample(
                    time_s=time_s,
                    packet_counter=packet_counter,
                    latitude_deg=latitude,
                    longitude_deg=longitude,
                    altitude_m=altitude,
                    epoch_s=epoch_s,
                )
            except (IndexError, OverflowError, TypeError, ValueError):
                continue

            if len(samples) >= MAX_REPLAY_SAMPLES:
                raise ValueError(
                    f"Telemetry CSV exceeds the {MAX_REPLAY_SAMPLES:,}-sample limit"
                )
            samples.append(sample)
    except csv.Error as exc:
        raise ValueError(f"Telemetry CSV could not be parsed: {exc}") from exc

    if not samples:
        raise ValueError(
            "No valid telemetry rows found; latitude and longitude must be in range "
            "and all telemetry values must be finite numbers"
        )
    return samples


def _validated_export_values(point: object) -> tuple[float, int, float, float, float, float | None]:
    try:
        time_s = _parse_number(getattr(point, "time_s"))
        counter = _parse_counter(getattr(point, "packet_counter"))
        latitude = _parse_number(getattr(point, "latitude_deg"))
        longitude = _parse_number(getattr(point, "longitude_deg"))
        altitude = _parse_number(getattr(point, "altitude_m"))
        raw_epoch = getattr(point, "epoch_s", None)
        epoch_s = None if raw_epoch is None else _parse_number(raw_epoch)
    except AttributeError as exc:
        missing_name = getattr(exc, "name", None)
        detail = f" {missing_name!r}" if missing_name else ""
        raise ValueError(f"Telemetry point is missing a required field{detail}") from exc

    if not -90.0 <= latitude <= 90.0:
        raise ValueError("Telemetry point latitude is outside [-90, 90]")
    if not -180.0 <= longitude <= 180.0:
        raise ValueError("Telemetry point longitude is outside [-180, 180]")
    if not _MIN_PLAUSIBLE_ALTITUDE_M <= altitude <= _MAX_PLAUSIBLE_ALTITUDE_M:
        raise ValueError("Telemetry point altitude is outside the plausible range")
    if abs(latitude) < 1e-9 and abs(longitude) < 1e-9:
        raise ValueError("Telemetry point does not contain a valid GNSS fix")
    return time_s, counter, latitude, longitude, altitude, epoch_s


def _format_float(value: float) -> str:
    return repr(float(value))


def telemetry_points_to_csv(points: Iterable[object]) -> str:
    """Serialize ReplaySample or TelemetryPoint-like objects as replayable CSV."""

    output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(
        (
            "time_s",
            "packet_counter",
            "latitude_deg",
            "longitude_deg",
            "altitude_m",
            "epoch_s",
        )
    )
    for point in points:
        time_s, counter, latitude, longitude, altitude, epoch_s = _validated_export_values(point)
        writer.writerow(
            (
                _format_float(time_s),
                str(counter),
                _format_float(latitude),
                _format_float(longitude),
                _format_float(altitude),
                "" if epoch_s is None else _format_float(epoch_s),
            )
        )
    return output.getvalue()


__all__ = [
    "MAX_CSV_BYTES",
    "MAX_REPLAY_SAMPLES",
    "ReplaySample",
    "parse_telemetry_csv",
    "telemetry_points_to_csv",
]
