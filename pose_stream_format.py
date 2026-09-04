"""Compact TCP pose frames for apps.

Internal fusion is Y-up (+X right, +Z forward). The app stream remaps to
Z-up (+X right, +Y forward) by swapping Y and Z. Webhook payloads are unchanged.
"""

from __future__ import annotations

from typing import Any

PROTOCOL = "raedir.pose.ndjson.v3"

# Position: 0.1 mm. Quaternion: ~0.005°.
_POS_DP = 4
_QUAT_DP = 5


def _r(value: float, digits: int) -> float:
    out = round(float(value), digits)
    return 0.0 if out == 0.0 else out


def swap_yz_vec(vec: dict[str, Any] | None) -> list[float]:
    """Internal (x, y-up, z-forward) -> app (x, y-forward, z-up)."""
    src = vec or {}
    return [
        _r(src.get("x", 0.0), _POS_DP),
        _r(src.get("z", 0.0), _POS_DP),
        _r(src.get("y", 0.0), _POS_DP),
    ]


def swap_yz_quat(quat: dict[str, Any] | None) -> list[float]:
    """Conjugate attitude by the Y/Z swap so the cube orients in the app frame.

    Positions use P=(x,z,y). Rotation becomes R' = P R P, which is
    q' = (w, -x, -z, -y) for q = (w, x, y, z).
    """
    src = quat or {}
    w = float(src.get("w", 1.0))
    x = float(src.get("x", 0.0))
    y = float(src.get("y", 0.0))
    z = float(src.get("z", 0.0))
    return [
        _r(w, _QUAT_DP),
        _r(-x, _QUAT_DP),
        _r(-z, _QUAT_DP),
        _r(-y, _QUAT_DP),
    ]


def _position(pose: dict[str, Any] | None) -> list[float] | None:
    if not pose or pose.get("position_m") is None:
        return None
    return swap_yz_vec(pose.get("position_m"))


def _rotation(pose: dict[str, Any] | None) -> list[float] | None:
    if not pose or pose.get("rotation") is None:
        return None
    return swap_yz_quat(pose.get("rotation"))


def hello_payload(ts_ms: int) -> dict[str, Any]:
    return {
        "type": "hello",
        "protocol": PROTOCOL,
        "axes": "x=right,y=forward,z=up",
        "t": ts_ms,
    }


def compact_app_frame(payload: dict[str, Any]) -> dict[str, Any]:
    """Minimal NDJSON: time, floor, dual position (p), dual rotation (r)."""
    pose = payload.get("pose") if isinstance(payload.get("pose"), dict) else None
    raw = payload.get("pose_raw") if isinstance(payload.get("pose_raw"), dict) else None
    ts_us = 0
    if pose is not None:
        ts_us = int(pose.get("timestamp_us") or 0)
    if not ts_us:
        ts_us = int(payload.get("updated_at_ms") or 0) * 1000

    frame: dict[str, Any] = {
        "t": ts_us,
        "f": _r(payload.get("floor_offset_m") or 0.0, 3),
        "n": int(payload.get("frame_seq") or 0),
        "s": 1 if payload.get("streaming") else 0,
    }

    filt_pos = _position(pose)
    raw_pos = _position(raw)
    if filt_pos is not None or raw_pos is not None:
        frame["p"] = (filt_pos or [0.0, 0.0, 0.0]) + (raw_pos or [0.0, 0.0, 0.0])

    filt_rot = _rotation(pose)
    raw_rot = _rotation(raw)
    if filt_rot is not None or raw_rot is not None:
        frame["r"] = (filt_rot or [1.0, 0.0, 0.0, 0.0]) + (raw_rot or [1.0, 0.0, 0.0, 0.0])

    return frame
