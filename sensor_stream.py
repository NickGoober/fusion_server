"""
Sensor stream protocol and time-aligned interpolation buffer.

Wire format (one JSON array per line):
  [sensor_index, timestamp, payload]

Sensor indices:
  0 — IMU linear acceleration (m/s²)  {"x","y","z"}
  1 — IMU game rotation quaternion     {"w","x","y","z"}
  2 — Optical flow                    {"dx","dy","quality"}
  3 — Radar range                     {"mm"}

Timestamp may be microseconds or milliseconds (values < 1e12 are treated as ms).
"""

from __future__ import annotations

import json
import math
import threading
from dataclasses import dataclass, field
from typing import Any, Callable

SENSOR_ACCEL = 0
SENSOR_QUAT = 1
SENSOR_FLOW = 2
SENSOR_RADAR = 3

DEFAULT_QUAT = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
DEFAULT_ACCEL = {"x": 0.0, "y": 0.0, "z": 0.0}
DEFAULT_FLOW = {"dx": 0, "dy": 0, "quality": 0}
DEFAULT_RANGE_MM = 550


def normalize_timestamp_us(ts: int | float) -> int:
    t = int(ts)
    if t < 1_000_000_000_000:
        return t * 1000
    return t


def format_sample(sensor: int, ts: int | float, data: dict[str, Any]) -> str:
    return json.dumps([sensor, ts, data], separators=(",", ":"))


def parse_sample_line(line: str) -> tuple[int, int, dict[str, Any]] | None:
    line = line.strip()
    if not line or line[0] not in "[{":
        return None
    try:
        raw = json.loads(line)
    except json.JSONDecodeError:
        return None

    if isinstance(raw, list) and len(raw) == 3:
        sensor = int(raw[0])
        ts_us = normalize_timestamp_us(raw[1])
        data = raw[2]
        if not isinstance(data, dict):
            return None
        return sensor, ts_us, data

    if isinstance(raw, dict) and "sensor" in raw:
        sensor = int(raw["sensor"])
        ts_key = "ts_us" if "ts_us" in raw else "t"
        ts_us = normalize_timestamp_us(raw.get(ts_key, raw.get("t", 0)))
        data = raw.get("data", raw.get("payload", {}))
        if not isinstance(data, dict):
            return None
        return sensor, ts_us, data

    return None


def is_control_message(line: str) -> bool:
    line = line.strip()
    return line.startswith("{") and '"type"' in line


@dataclass
class TimedSample:
    ts_us: int
    value: Any


def _quat_normalize(q: dict[str, float]) -> dict[str, float]:
    n = math.sqrt(q["w"] ** 2 + q["x"] ** 2 + q["y"] ** 2 + q["z"] ** 2)
    if n < 1e-12:
        return dict(DEFAULT_QUAT)
    return {k: q[k] / n for k in q}


def _quat_conj(q: dict[str, float]) -> dict[str, float]:
    return {"w": q["w"], "x": -q["x"], "y": -q["y"], "z": -q["z"]}


def _quat_mult(a: dict[str, float], b: dict[str, float]) -> dict[str, float]:
    return {
        "w": a["w"] * b["w"] - a["x"] * b["x"] - a["y"] * b["y"] - a["z"] * b["z"],
        "x": a["w"] * b["x"] + a["x"] * b["w"] + a["y"] * b["z"] - a["z"] * b["y"],
        "y": a["w"] * b["y"] - a["x"] * b["z"] + a["y"] * b["w"] + a["z"] * b["x"],
        "z": a["w"] * b["z"] + a["x"] * b["y"] - a["y"] * b["x"] + a["z"] * b["w"],
    }


def _slerp_quat(
    lo: dict[str, float], hi: dict[str, float], alpha: float,
) -> dict[str, float]:
    q0 = _quat_normalize(lo)
    q1 = _quat_normalize(hi)
    dot = q0["w"] * q1["w"] + q0["x"] * q1["x"] + q0["y"] * q1["y"] + q0["z"] * q1["z"]
    if dot < 0.0:
        q1 = {k: -v for k, v in q1.items()}
        dot = -dot
    if dot > 0.9995:
        out = {k: q0[k] + alpha * (q1[k] - q0[k]) for k in q0}
        return _quat_normalize(out)
    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    return _quat_normalize({
        "w": s0 * q0["w"] + s1 * q1["w"],
        "x": s0 * q0["x"] + s1 * q1["x"],
        "y": s0 * q0["y"] + s1 * q1["y"],
        "z": s0 * q0["z"] + s1 * q1["z"],
    })


def _lerp(a: float, b: float, alpha: float) -> float:
    return a + (b - a) * alpha


def _lerp_vec3(lo: dict[str, float], hi: dict[str, float], alpha: float) -> dict[str, float]:
    return {
        "x": _lerp(lo["x"], hi["x"], alpha),
        "y": _lerp(lo["y"], hi["y"], alpha),
        "z": _lerp(lo["z"], hi["z"], alpha),
    }


def _bracket(samples: list[TimedSample], ts_us: int) -> tuple[TimedSample | None, TimedSample | None]:
    if not samples:
        return None, None
    if ts_us <= samples[0].ts_us:
        return samples[0], samples[0]
    if ts_us >= samples[-1].ts_us:
        return samples[-1], samples[-1]
    lo = samples[0]
    hi = samples[-1]
    for i in range(len(samples) - 1):
        if samples[i].ts_us <= ts_us <= samples[i + 1].ts_us:
            lo = samples[i]
            hi = samples[i + 1]
            break
    return lo, hi


def _interp_vec3_channel(
    samples: list[TimedSample], ts_us: int, default: dict[str, float],
) -> dict[str, float]:
    lo, hi = _bracket(samples, ts_us)
    if lo is None:
        return dict(default)
    if lo is hi or lo.ts_us == hi.ts_us:
        return dict(lo.value)
    alpha = (ts_us - lo.ts_us) / (hi.ts_us - lo.ts_us)
    return _lerp_vec3(lo.value, hi.value, alpha)


def _interp_quat_channel(
    samples: list[TimedSample], ts_us: int, default: dict[str, float],
) -> dict[str, float]:
    lo, hi = _bracket(samples, ts_us)
    if lo is None:
        return dict(default)
    if lo is hi or lo.ts_us == hi.ts_us:
        return dict(lo.value)
    alpha = (ts_us - lo.ts_us) / (hi.ts_us - lo.ts_us)
    return _slerp_quat(lo.value, hi.value, alpha)


def _interp_scalar_channel(samples: list[TimedSample], ts_us: int, default: float) -> float:
    lo, hi = _bracket(samples, ts_us)
    if lo is None:
        return default
    if lo is hi or lo.ts_us == hi.ts_us:
        return float(lo.value)
    alpha = (ts_us - lo.ts_us) / (hi.ts_us - lo.ts_us)
    return _lerp(float(lo.value), float(hi.value), alpha)


def _flow_in_interval(
    samples: list[TimedSample], t_lo_us: int, t_hi_us: int,
) -> dict[str, int]:
    dx = dy = quality = 0
    found = False
    for sample in samples:
        if t_lo_us < sample.ts_us <= t_hi_us:
            val = sample.value
            dx += int(val["dx"])
            dy += int(val["dy"])
            quality = max(quality, int(val.get("quality", 0)))
            found = True
    if not found:
        return dict(DEFAULT_FLOW)
    return {"dx": dx, "dy": dy, "quality": quality}


def gyro_from_quat_pair(
    q_prev: dict[str, float],
    q_curr: dict[str, float],
    dt_s: float,
) -> dict[str, float]:
    if dt_s <= 1e-6:
        return dict(DEFAULT_ACCEL)
    q_rel = _quat_mult(_quat_normalize(q_curr), _quat_conj(_quat_normalize(q_prev)))
    w = max(-1.0, min(1.0, q_rel["w"]))
    angle = 2.0 * math.acos(w)
    if angle < 1e-8:
        return dict(DEFAULT_ACCEL)
    sin_half = math.sin(angle * 0.5)
    if abs(sin_half) < 1e-8:
        return dict(DEFAULT_ACCEL)
    scale = angle / (sin_half * dt_s)
    return {
        "x": q_rel["x"] * scale,
        "y": q_rel["y"] * scale,
        "z": q_rel["z"] * scale,
    }


@dataclass
class SensorStreamBuffer:
    """Buffers heterogeneous sensor samples and emits regular fused ticks."""

    latency_us: int = 4_000_000
    output_hz: float = 100.0
    on_tick: Callable[[dict[str, Any]], None] | None = None

    accel: list[TimedSample] = field(default_factory=list)
    quat: list[TimedSample] = field(default_factory=list)
    flow: list[TimedSample] = field(default_factory=list)
    radar: list[TimedSample] = field(default_factory=list)

    _latest_ts_us: int = 0
    _next_emit_us: int | None = None
    _prev_quat: dict[str, float] | None = None
    _prev_quat_ts_us: int | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def reset(self) -> None:
        with self._lock:
            self.accel.clear()
            self.quat.clear()
            self.flow.clear()
            self.radar.clear()
            self._latest_ts_us = 0
            self._next_emit_us = None
            self._prev_quat = None
            self._prev_quat_ts_us = None

    def ingest(self, sensor: int, ts_us: int, data: dict[str, Any]) -> None:
        with self._lock:
            if sensor == SENSOR_ACCEL:
                self.accel.append(TimedSample(ts_us, {
                    "x": float(data["x"]), "y": float(data["y"]), "z": float(data["z"]),
                }))
            elif sensor == SENSOR_QUAT:
                q = {
                    "w": float(data["w"]), "x": float(data["x"]),
                    "y": float(data["y"]), "z": float(data["z"]),
                }
                self.quat.append(TimedSample(ts_us, _quat_normalize(q)))
            elif sensor == SENSOR_FLOW:
                self.flow.append(TimedSample(ts_us, {
                    "dx": int(data.get("dx", data.get("delta_x", 0))),
                    "dy": int(data.get("dy", data.get("delta_y", 0))),
                    "quality": int(data.get("quality", 255)),
                }))
            elif sensor == SENSOR_RADAR:
                filtered = data.get("filtered")
                if filtered:
                    mm = int(filtered.get("distance_mm", data.get("mm", DEFAULT_RANGE_MM)))
                else:
                    mm = int(data.get("mm", data.get("distance_mm", DEFAULT_RANGE_MM)))
                self.radar.append(TimedSample(ts_us, mm))
            else:
                return

            for channel in (self.accel, self.quat, self.flow, self.radar):
                channel.sort(key=lambda s: s.ts_us)

            self._latest_ts_us = max(self._latest_ts_us, ts_us)
            self._drain_locked()

    def _drain_locked(self) -> None:
        if self.on_tick is None:
            return
        dt_us = int(1_000_000 / self.output_hz)
        ready_until = self._latest_ts_us - self.latency_us
        if ready_until <= 0:
            return

        if self._next_emit_us is None:
            if not self.quat and not self.accel:
                return
            earliest = min(
                (self.accel[0].ts_us if self.accel else 2**62),
                (self.quat[0].ts_us if self.quat else 2**62),
            )
            self._next_emit_us = earliest + self.latency_us

        prev_flow_lo = self._next_emit_us - dt_us
        while self._next_emit_us <= ready_until:
            tick = self._bundle_at_locked(self._next_emit_us, prev_flow_lo)
            if tick is not None:
                self.on_tick(tick)
            prev_flow_lo = self._next_emit_us
            self._next_emit_us += dt_us

    def _bundle_at_locked(self, ts_us: int, flow_lo_us: int) -> dict[str, Any] | None:
        quat = _interp_quat_channel(self.quat, ts_us, DEFAULT_QUAT)
        accel = _interp_vec3_channel(self.accel, ts_us, DEFAULT_ACCEL)
        flow = _flow_in_interval(self.flow, flow_lo_us, ts_us)
        range_mm = int(round(_interp_scalar_channel(
            self.radar, ts_us, float(DEFAULT_RANGE_MM),
        )))

        if self._prev_quat is not None and self._prev_quat_ts_us is not None:
            dt_s = (ts_us - self._prev_quat_ts_us) / 1_000_000.0
            gyro = gyro_from_quat_pair(self._prev_quat, quat, dt_s)
        else:
            gyro = dict(DEFAULT_ACCEL)

        self._prev_quat = quat
        self._prev_quat_ts_us = ts_us

        return {
            "type": "sensor",
            "ts_us": ts_us,
            "quat": quat,
            "gyro": gyro,
            "accel": accel,
            "flow": flow,
            "range": {"mm": range_mm},
        }

    def flush(self) -> None:
        """Emit any remaining ticks up to latest sample time."""
        with self._lock:
            old_latency = self.latency_us
            self.latency_us = 0
            self._drain_locked()
            self.latency_us = old_latency
