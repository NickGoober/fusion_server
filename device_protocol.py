"""
Convert collar device JSONL rows to fusion sensor-stream samples.

Device rows (one JSON object per line) use async single-sensor messages:

  {"kind":"quat","t_ms":1234,"quat":{"w":1,"x":0,"y":0,"z":0}}
  {"kind":"accel","t_ms":1234,"accel_mps2":{"x":0,"y":0,"z":9.8}}
  {"kind":"flow","t_ms":1234,"flow":{"delta_x":1,"delta_y":0,"quality":255}}
  {"kind":"range","t_ms":1234,"filtered":{"distance_mm":550,"valid":true}}

Also accepts pre-encoded stream lines: [sensor_type, timestamp, data_array]

Collar firmware legacy mapping (differs from the generic table above):
  0 — game rotation quaternion [x, y, z, w]
  1 — BNO085 linear acceleration (gravity removed) [x, y, z] m/s²
  2 — optical flow [dx, dy, quality]
  3 — radar range [mm, ...]
"""

from __future__ import annotations

import json
from typing import Any

from sensor_stream import (
    SENSOR_ACCEL,
    SENSOR_FLOW,
    SENSOR_QUAT,
    SENSOR_RADAR,
    normalize_timestamp_us,
    parse_sample_line,
)


def _collar_legacy_array_to_samples(
    sensor: int,
    ts_us: int,
    arr: list[Any],
) -> list[tuple[int, int, dict[str, Any]]]:
    """Map collar wire arrays (0=quat, 1=linear accel) to internal sensor types."""
    if sensor == 0 and len(arr) >= 4:
        return [(
            SENSOR_QUAT,
            ts_us,
            {
                "x": float(arr[0]),
                "y": float(arr[1]),
                "z": float(arr[2]),
                "w": float(arr[3]),
            },
        )]
    if sensor == 1 and len(arr) == 3:
        return [(
            SENSOR_ACCEL,
            ts_us,
            {"x": float(arr[0]), "y": float(arr[1]), "z": float(arr[2])},
        )]
    if sensor == 2 and len(arr) >= 2:
        quality = int(arr[2]) if len(arr) >= 3 else 255
        return [(
            SENSOR_FLOW,
            ts_us,
            {"dx": int(arr[0]), "dy": int(arr[1]), "quality": quality},
        )]
    if sensor == 3 and len(arr) >= 1:
        return [(SENSOR_RADAR, ts_us, {"mm": int(arr[0])})]
    return []


def _vec3_from_row(row: dict, *keys: str) -> dict[str, float] | None:
    for key in keys:
        raw = row.get(key)
        if raw is None:
            continue
        return {"x": float(raw["x"]), "y": float(raw["y"]), "z": float(raw["z"])}
    return None


def _quat_from_row(row: dict) -> dict[str, float] | None:
    raw = row.get("quat")
    if raw is None:
        return None
    return {
        "w": float(raw["w"]),
        "x": float(raw["x"]),
        "y": float(raw["y"]),
        "z": float(raw["z"]),
    }


def timestamp_us_from_row(row: dict, *, host_ts_us: int | None = None) -> int:
    if "ts_us" in row:
        return normalize_timestamp_us(row["ts_us"])
    if "t_ms" in row:
        return int(float(row["t_ms"]) * 1000)
    if "sim_time_s" in row:
        return int(float(row["sim_time_s"]) * 1_000_000)
    if host_ts_us is not None:
        return host_ts_us
    raise KeyError("row missing ts_us, t_ms, or sim_time_s")


def device_row_to_stream_samples(
    row: dict,
    *,
    host_ts_us: int | None = None,
) -> list[tuple[int, int, dict[str, Any]]]:
    """Map one device JSONL object to zero or more (sensor_index, ts_us, payload) tuples."""
    kind = row.get("kind")
    if kind is None and isinstance(row.get("sensor"), int):
        ts_us = timestamp_us_from_row(row, host_ts_us=host_ts_us)
        payload = row.get("data", row.get("payload"))
        if isinstance(payload, list):
            from sensor_stream import payload_array_to_dict
            converted = payload_array_to_dict(int(row["sensor"]), payload)
            if converted is not None:
                sensor, data = converted
                return [(sensor, ts_us, data)]
        if isinstance(payload, dict):
            return [(int(row["sensor"]), ts_us, payload)]

    try:
        ts_us = timestamp_us_from_row(row, host_ts_us=host_ts_us)
    except KeyError:
        return []

    if kind == "quat":
        quat = _quat_from_row(row)
        return [(SENSOR_QUAT, ts_us, quat)] if quat else []

    if kind == "accel":
        vec = _vec3_from_row(row, "accel_mps2", "accel_ms2", "accel")
        return [(SENSOR_ACCEL, ts_us, vec)] if vec else []

    if kind == "flow":
        flow = row.get("flow")
        if flow is None:
            return []
        return [(
            SENSOR_FLOW,
            ts_us,
            {
                "dx": int(flow.get("dx", flow.get("delta_x", 0))),
                "dy": int(flow.get("dy", flow.get("delta_y", 0))),
                "quality": int(flow.get("quality", 255)),
            },
        )]

    if kind == "range":
        filtered = row.get("filtered")
        if filtered is not None:
            if not filtered.get("valid", True):
                return []
            return [(SENSOR_RADAR, ts_us, {"mm": int(filtered["distance_mm"])})]

        rng = row.get("range")
        if rng is None:
            return []
        if not rng.get("valid", True):
            return []
        return [(SENSOR_RADAR, ts_us, {"mm": int(rng["mm"])})]

    return []


def collar_line_to_stream_samples(
    line: str,
    *,
    host_ts_us: int | None = None,
) -> list[tuple[int, int, dict[str, Any]]]:
    """Parse one collar line (stream array or device JSONL) into stream samples."""
    line = line.strip()
    if not line:
        return []

    parsed = parse_sample_line(line)
    if parsed is not None:
        sensor, ts_us, data = parsed
        return [(sensor, ts_us, data)]

    line_stripped = line.strip()
    if line_stripped.startswith("["):
        try:
            raw = json.loads(line_stripped)
        except json.JSONDecodeError:
            raw = None
        if isinstance(raw, list) and len(raw) == 3 and isinstance(raw[2], list):
            legacy = _collar_legacy_array_to_samples(
                int(raw[0]),
                normalize_timestamp_us(raw[1]),
                raw[2],
            )
            if legacy:
                return legacy

    if not line.startswith("{"):
        return []

    try:
        row = json.loads(line)
    except json.JSONDecodeError:
        return []

    if not isinstance(row, dict):
        return []

    return device_row_to_stream_samples(row, host_ts_us=host_ts_us)
