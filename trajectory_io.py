"""Dependency-free trajectory CSV parsing for the flight viewer.

The parser deliberately accepts a small family of semantic formats instead of
depending on one exporter-specific column order.  Header names are normalized,
so common unit suffixes (``East (m)``, ``latitude_deg`` and similar) do not
affect format detection.
"""

from __future__ import annotations

import csv
import math
import re
import unicodedata
from dataclasses import dataclass
from pathlib import PurePath
from typing import Optional


@dataclass(frozen=True)
class TrajectoryRecord:
    """One imported trajectory sample, expressed as ENU or geodetic data."""

    time_s: Optional[float] = None
    east_m: Optional[float] = None
    north_m: Optional[float] = None
    up_m: Optional[float] = None
    latitude_deg: Optional[float] = None
    longitude_deg: Optional[float] = None
    altitude_m: Optional[float] = None


@dataclass(frozen=True)
class LoadedTrajectory:
    """A parsed trajectory and the coordinate mode used by its records."""

    name: str
    records: tuple[TrajectoryRecord, ...]
    mode: str


@dataclass(frozen=True)
class _Column:
    index: int
    factor: float


@dataclass(frozen=True)
class _HeaderMatch:
    line_index: int
    delimiter: str
    mode: str
    columns: dict[str, _Column]
    score: tuple[int, int, int]


_DELIMITERS = (",", ";", "\t")
_MISSING_VALUES = {"", "-", "--", "n/a", "na", "null", "none"}
MAX_CSV_BYTES = 16 * 1024 * 1024
MAX_TRAJECTORY_POINTS = 50_000
_MAX_HEADER_SCAN_LINES = 200


def _ascii_words(value: str, *, remove_units: bool) -> str:
    value = value.strip().lstrip("\ufeff").casefold()
    value = (
        value.replace("ı", "i")
        .replace("°", " deg ")
        .replace("º", " deg ")
        .replace("μ", "u")
        .replace("µ", "u")
    )
    value = unicodedata.normalize("NFKD", value)
    value = "".join(character for character in value if not unicodedata.combining(character))
    if remove_units:
        value = re.sub(r"\([^)]*\)|\[[^]]*]", " ", value)
    value = re.sub(r"[^a-z0-9]+", " ", value)
    words = value.split()
    if remove_units:
        unit_words = {
            "s",
            "sec",
            "secs",
            "second",
            "seconds",
            "ms",
            "millisecond",
            "milliseconds",
            "min",
            "mins",
            "minute",
            "minutes",
            "m",
            "meter",
            "meters",
            "metre",
            "metres",
            "km",
            "kilometer",
            "kilometers",
            "kilometre",
            "kilometres",
            "cm",
            "mm",
            "ft",
            "foot",
            "feet",
            "deg",
            "degree",
            "degrees",
            "rad",
            "radian",
            "radians",
        }
        words = [word for word in words if word not in unit_words]
    return " ".join(words)


def _normalise_header(value: str) -> str:
    return _ascii_words(value, remove_units=True)


_ALIASES = {
    "time": {
        "t",
        "time",
        "flight time",
        "elapsed time",
        "simulation time",
        "timestamp",
        "zaman",
        "sure",
        "ucus zamani",
        "gecen sure",
    },
    "east": {
        "e",
        "east",
        "easting",
        "east position",
        "position east",
        "position east of launch",
        "east displacement",
        "east distance",
        "x",
        "x position",
        "position x",
        "dogu",
        "dogu konumu",
    },
    "north": {
        "n",
        "north",
        "northing",
        "north position",
        "position north",
        "position north of launch",
        "north displacement",
        "north distance",
        "y",
        "y position",
        "position y",
        "kuzey",
        "kuzey konumu",
    },
    "up": {
        "u",
        "up",
        "up position",
        "vertical position",
        "vertical displacement",
        "z",
        "z position",
        "position z",
        "yukari",
        "yukari konumu",
    },
    "latitude": {
        "lat",
        "latitude",
        "gps latitude",
        "enlem",
    },
    "longitude": {
        "lon",
        "long",
        "lng",
        "longitude",
        "gps longitude",
        "boylam",
    },
    "altitude": {
        "alt",
        "altitude",
        "gps altitude",
        "altitude asl",
        "altitude agl",
        "height",
        "elevation",
        "irtifa",
        "rakim",
        "yukseklik",
    },
    "lateral": {
        "lateral",
        "lateral distance",
        "downrange",
        "down range",
        "downrange distance",
        "horizontal distance",
        "ground distance",
        "range",
        "yanal mesafe",
        "yatay mesafe",
        "menzil",
    },
}

_ALIAS_TO_FIELD = {
    _normalise_header(alias): field
    for field, aliases in _ALIASES.items()
    for alias in aliases
}


def _canonical_field(header: str) -> Optional[str]:
    return _ALIAS_TO_FIELD.get(_normalise_header(header))


def _unit_factor(header: str, field: str) -> float:
    """Return a multiplier that converts a supported header unit to SI."""
    words = set(_ascii_words(header, remove_units=False).split())

    if field == "time":
        if words & {"ms", "millisecond", "milliseconds"}:
            return 0.001
        if words & {"min", "mins", "minute", "minutes"}:
            return 60.0
        return 1.0

    if field in {"latitude", "longitude"}:
        if words & {"rad", "radian", "radians"}:
            return 180.0 / math.pi
        return 1.0

    if words & {"km", "kilometer", "kilometers", "kilometre", "kilometres"}:
        return 1000.0
    if words & {"cm"}:
        return 0.01
    if words & {"mm"}:
        return 0.001
    if words & {"ft", "foot", "feet"}:
        return 0.3048
    return 1.0


def _decode_csv(data: str | bytes) -> str:
    if isinstance(data, str):
        if len(data) > MAX_CSV_BYTES:
            raise ValueError("Trajectory CSV exceeds the 16 MiB size limit")
        return data.lstrip("\ufeff")
    if not isinstance(data, bytes):
        raise TypeError("Trajectory CSV data must be str or bytes")
    if len(data) > MAX_CSV_BYTES:
        raise ValueError("Trajectory CSV exceeds the 16 MiB size limit")

    try:
        return data.decode("utf-8-sig")
    except UnicodeDecodeError:
        try:
            return data.decode("cp1254").lstrip("\ufeff")
        except UnicodeDecodeError as exc:
            raise ValueError(
                "Trajectory CSV could not be decoded as UTF-8 or Windows-1254"
            ) from exc


def _read_row(line: str, delimiter: str) -> list[str]:
    return next(csv.reader([line], delimiter=delimiter, skipinitialspace=True))


def _header_text(line: str) -> str:
    stripped = line.lstrip()
    if stripped.startswith("#"):
        return stripped[1:].lstrip()
    return line


def _detect_mode(columns: dict[str, _Column]) -> Optional[str]:
    fields = set(columns)
    if {"latitude", "longitude", "altitude"} <= fields:
        return "llh"
    if {"east", "north"} <= fields and ({"up"} <= fields or {"altitude"} <= fields):
        return "enu"
    if {"time", "altitude", "lateral"} <= fields:
        return "openrocket_2d"
    return None


def _find_header(lines: list[str]) -> _HeaderMatch:
    best: Optional[_HeaderMatch] = None
    for line_index, line in enumerate(lines[:_MAX_HEADER_SCAN_LINES]):
        candidate = _header_text(line)
        if not candidate.strip():
            continue

        for delimiter in _DELIMITERS:
            try:
                cells = _read_row(candidate, delimiter)
            except csv.Error:
                continue
            if len(cells) < 3:
                continue

            columns: dict[str, _Column] = {}
            for index, cell in enumerate(cells):
                field = _canonical_field(cell)
                if field is not None and field not in columns:
                    columns[field] = _Column(index=index, factor=_unit_factor(cell, field))

            mode = _detect_mode(columns)
            if mode is None:
                continue

            match = _HeaderMatch(
                line_index=line_index,
                delimiter=delimiter,
                mode=mode,
                columns=columns,
                score=(len(columns), len(cells), -line_index),
            )
            if best is None or match.score > best.score:
                best = match

    if best is None:
        raise ValueError(
            "Trajectory CSV header not recognized; expected East/North/Up, "
            "Latitude/Longitude/Altitude, or Time/Altitude/Lateral distance "
            "(Downrange) columns"
        )
    return best


def _number(value: str, delimiter: str) -> float:
    text = value.strip().replace("\u00a0", "").replace("\u202f", "").replace(" ", "")
    if text.casefold() in _MISSING_VALUES:
        raise ValueError("missing numeric value")

    if delimiter != "," and "," in text:
        if "." not in text:
            text = text.replace(",", ".")
        elif text.rfind(",") > text.rfind("."):
            text = text.replace(".", "").replace(",", ".")
        else:
            text = text.replace(",", "")

    value_float = float(text)
    if not math.isfinite(value_float):
        raise ValueError("numeric value is not finite")
    return value_float


def _column_value(
    row: list[str],
    match: _HeaderMatch,
    field: str,
    *,
    required: bool,
) -> Optional[float]:
    column = match.columns.get(field)
    if column is None:
        if required:
            raise ValueError(f"missing required {field} column")
        return None
    if column.index >= len(row) or row[column.index].strip().casefold() in _MISSING_VALUES:
        if required:
            raise ValueError(f"missing required {field} value")
        return None
    return _number(row[column.index], match.delimiter) * column.factor


def _record_from_row(row: list[str], match: _HeaderMatch) -> TrajectoryRecord:
    time_s = _column_value(row, match, "time", required=False)

    if match.mode == "enu":
        vertical_field = "up" if "up" in match.columns else "altitude"
        return TrajectoryRecord(
            time_s=time_s,
            east_m=_column_value(row, match, "east", required=True),
            north_m=_column_value(row, match, "north", required=True),
            up_m=_column_value(row, match, vertical_field, required=True),
        )

    if match.mode == "llh":
        latitude = _column_value(row, match, "latitude", required=True)
        longitude = _column_value(row, match, "longitude", required=True)
        altitude = _column_value(row, match, "altitude", required=True)
        assert latitude is not None and longitude is not None and altitude is not None
        if not -90.0 <= latitude <= 90.0:
            raise ValueError("latitude is outside valid range")
        if not -180.0 <= longitude <= 180.0:
            raise ValueError("longitude is outside valid range")
        return TrajectoryRecord(
            time_s=time_s,
            latitude_deg=latitude,
            longitude_deg=longitude,
            altitude_m=altitude,
        )

    return TrajectoryRecord(
        time_s=_column_value(row, match, "time", required=True),
        east_m=_column_value(row, match, "lateral", required=True),
        north_m=0.0,
        up_m=_column_value(row, match, "altitude", required=True),
    )


def _trajectory_name(filename: str) -> str:
    # Replacing backslashes also makes Windows-style upload names predictable
    # when this module is exercised on another operating system.
    basename = PurePath(str(filename).replace("\\", "/")).name
    stem = PurePath(basename).stem.strip()
    return stem or "trajectory"


def parse_trajectory_csv(
    data: str | bytes,
    filename: str = "trajectory.csv",
) -> LoadedTrajectory:
    """Parse an ENU, LLH or OpenRocket two-dimensional trajectory CSV.

    Blank lines, comment lines and malformed data rows are ignored.  A file is
    accepted only when at least two complete, finite records remain.
    """
    text = _decode_csv(data)
    lines = text.splitlines()
    match = _find_header(lines)

    records: list[TrajectoryRecord] = []
    for line in lines[match.line_index + 1 :]:
        stripped = line.lstrip()
        if not stripped or stripped.startswith("#"):
            continue
        try:
            row = _read_row(line, match.delimiter)
            record = _record_from_row(row, match)
        except (csv.Error, IndexError, ValueError):
            continue
        if len(records) >= MAX_TRAJECTORY_POINTS:
            raise ValueError(
                f"Trajectory CSV exceeds the {MAX_TRAJECTORY_POINTS:,}-point limit"
            )
        records.append(record)

    if len(records) < 2:
        raise ValueError(
            "Trajectory CSV must contain at least 2 valid points; "
            f"found {len(records)}"
        )

    return LoadedTrajectory(
        name=_trajectory_name(filename),
        records=tuple(records),
        mode=match.mode,
    )


__all__ = [
    "LoadedTrajectory",
    "MAX_CSV_BYTES",
    "MAX_TRAJECTORY_POINTS",
    "TrajectoryRecord",
    "parse_trajectory_csv",
]
