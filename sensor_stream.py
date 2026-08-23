"""
Sensor stream protocol and time-aligned interpolation buffer.

Wire format (one JSON array per line, or a batch of rows per line):
  [sensor_type, timestamp, data]

  One-second collar batches:
  [[sensor_type, timestamp, data], ...]

  data is a plain JSON array (not an object).

Sensor types and data layouts:
  0 — IMU acceleration (m/s²)     [x, y, z]
      (also accepts [x, y, z, w] at type 0 → treated as quaternion)
  1 — IMU game rotation quaternion  [x, y, z, w]
  2 — Optical flow                  [dx, dy, quality]
  3 — Radar range (mm)              [mm]
  99 — Control command              [code]  (see parse_stream_command)

Plain-text commands (one line, no JSON) are also accepted:
  STREAM_START, STREAM_END
  ($ prefix optional, e.g. $STREAM_START)

Control codes for type 99:
  10 = stream_start, 11 = stream_end

Timestamp may be microseconds (collar micros since boot, simulation from 0) or
epoch milliseconds (~1e12+). Values in the legacy millisecond-uptime range are
no longer scaled automatically — prefer microsecond monotonic clocks on device.
"""

from __future__ import annotations

import json
import math
import statistics
import threading
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Any, Callable

SENSOR_ACCEL = 0
SENSOR_QUAT = 1
SENSOR_FLOW = 2
SENSOR_RADAR = 3
SENSOR_CONTROL = 99

CMD_STREAM_START = "stream_start"
CMD_STREAM_END = "stream_end"

_CONTROL_ARRAY_CODES: dict[int, str] = {
    10: CMD_STREAM_START,
    11: CMD_STREAM_END,
}

_TEXT_COMMANDS: dict[str, str] = {
    "STREAM_START": CMD_STREAM_START,
    "$STREAM_START": CMD_STREAM_START,
    "STREAM_END": CMD_STREAM_END,
    "$STREAM_END": CMD_STREAM_END,
}

DEFAULT_QUAT = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
DEFAULT_ACCEL = {"x": 0.0, "y": 0.0, "z": 0.0}
DEFAULT_FLOW = {"dx": 0, "dy": 0, "quality": 0}
DEFAULT_RANGE_MM = 550
# Do not SLERP quats across long gaps — hold last sample until the next arrives.
MAX_QUAT_SLERP_GAP_US = 100_000
# Cap quat-derived gyro spikes (bad SLERP gaps / noise blow up r = a/ω²).
MAX_QUAT_GYRO_RAD_S = 6.0
# Minimum dot product between consecutive quats to treat as a frozen duplicate.
QUAT_DEDUP_DOT_THRESHOLD = 0.99999
# Wide window for quat-derived gyro at stream output rate (seconds).
QUAT_GYRO_WINDOW_S = 0.15

# Collar firmware sends micros() since boot (~1e9+). Epoch ms is ~1.7e12+.
_EPOCH_MS_MIN = 1_000_000_000_000
_EPOCH_US_MIN = 1_000_000_000_000_000


def normalize_timestamp_us(ts: int | float) -> int:
    """Convert a wire-format timestamp to microseconds."""
    t = int(ts)
    if t >= _EPOCH_US_MIN:
        return t
    if t >= _EPOCH_MS_MIN:
        return t * 1000
    # Monotonic device clocks (micros since boot) and synthetic streams from t0.
    return t


def detect_timestamp_scale(raw_timestamps: list[int]) -> float:
    """
    Divisor to convert raw timestamp deltas to seconds for replay timing.

    Collar captures use micros since boot (~1e9) but were previously misread as
    milliseconds (1000x slower replay). Use span/avg heuristics when ambiguous.
    """
    if len(raw_timestamps) < 2:
        return 1_000_000.0
    sorted_ts = sorted(raw_timestamps)
    span = sorted_ts[-1] - sorted_ts[0]
    if span <= 0:
        return 1_000_000.0
    avg = span / (len(sorted_ts) - 1)
    if span >= 1_000_000_000 and avg < 1_000_000:
        return 1_000_000.0
    if avg < 200:
        return 1000.0
    return 1_000_000.0


def payload_array_to_dict(sensor: int, arr: list[Any]) -> tuple[int, dict[str, Any]] | None:
    """Convert wire-format data array to internal dict; returns (sensor, dict)."""
    if not arr:
        return None

    if sensor == SENSOR_ACCEL:
        if len(arr) >= 3 and len(arr) < 4:
            return sensor, {
                "x": float(arr[0]), "y": float(arr[1]), "z": float(arr[2]),
            }
        if len(arr) >= 4:
            # Collar firmware may send quaternion at type 0 as [x, y, z, w].
            return SENSOR_QUAT, {
                "x": float(arr[0]), "y": float(arr[1]),
                "z": float(arr[2]), "w": float(arr[3]),
            }

    if sensor == SENSOR_QUAT:
        if len(arr) >= 4:
            return sensor, {
                "x": float(arr[0]), "y": float(arr[1]),
                "z": float(arr[2]), "w": float(arr[3]),
            }
        if len(arr) == 3:
            return SENSOR_ACCEL, {
                "x": float(arr[0]), "y": float(arr[1]), "z": float(arr[2]),
            }

    if sensor == SENSOR_FLOW:
        if len(arr) >= 2:
            quality = int(arr[2]) if len(arr) >= 3 else 255
            return sensor, {
                "dx": int(arr[0]), "dy": int(arr[1]), "quality": quality,
            }

    if sensor == SENSOR_RADAR:
        if len(arr) >= 1:
            return sensor, {"mm": int(arr[0])}

    return None


def dict_to_payload_array(sensor: int, data: dict[str, Any]) -> list[Any]:
    """Convert internal dict to wire-format data array."""
    if sensor == SENSOR_ACCEL:
        return [float(data["x"]), float(data["y"]), float(data["z"])]
    if sensor == SENSOR_QUAT:
        return [
            float(data["x"]), float(data["y"]),
            float(data["z"]), float(data["w"]),
        ]
    if sensor == SENSOR_FLOW:
        return [
            int(data.get("dx", data.get("delta_x", 0))),
            int(data.get("dy", data.get("delta_y", 0))),
            int(data.get("quality", 255)),
        ]
    if sensor == SENSOR_RADAR:
        filtered = data.get("filtered")
        if filtered:
            return [int(filtered.get("distance_mm", data.get("mm", DEFAULT_RANGE_MM)))]
        return [int(data.get("mm", data.get("distance_mm", DEFAULT_RANGE_MM)))]
    raise ValueError(f"unknown sensor type {sensor}")


def format_sample(sensor: int, ts: int | float, data: dict[str, Any]) -> str:
    return json.dumps(
        [sensor, ts, dict_to_payload_array(sensor, data)],
        separators=(",", ":"),
    )


def format_sample_array(sensor: int, ts: int | float, data: list[Any]) -> str:
    return json.dumps([sensor, ts, data], separators=(",", ":"))


def parse_stream_command(line: str) -> str | None:
    """
    Parse a collar stream control command.

    Accepts plain text (STREAM_START) or wire array [99, timestamp, [code]].
    Returns command name or None if not a control line.
    """
    stripped = line.strip()
    if not stripped:
        return None

    text_key = stripped.upper()
    if text_key in _TEXT_COMMANDS:
        return _TEXT_COMMANDS[text_key]

    if stripped[0] != "[":
        return None

    try:
        raw = json.loads(stripped)
    except json.JSONDecodeError:
        return None

    if not isinstance(raw, list) or len(raw) != 3:
        return None
    if int(raw[0]) != SENSOR_CONTROL:
        return None

    payload = raw[2]
    if isinstance(payload, list) and payload:
        return _CONTROL_ARRAY_CODES.get(int(payload[0]))
    if isinstance(payload, (int, float)):
        return _CONTROL_ARRAY_CODES.get(int(payload))

    return None


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
        payload = raw[2]
        if isinstance(payload, list):
            converted = payload_array_to_dict(sensor, payload)
            if converted is None:
                return None
            sensor, data = converted
            return sensor, ts_us, data
        if isinstance(payload, dict):
            return sensor, ts_us, payload

    if isinstance(raw, dict) and "sensor" in raw:
        sensor = int(raw["sensor"])
        ts_key = "ts_us" if "ts_us" in raw else "t"
        ts_us = normalize_timestamp_us(raw.get(ts_key, raw.get("t", 0)))
        payload = raw.get("data", raw.get("payload"))
        if isinstance(payload, list):
            converted = payload_array_to_dict(sensor, payload)
            if converted is None:
                return None
            sensor, data = converted
            return sensor, ts_us, data
        if isinstance(payload, dict):
            return sensor, ts_us, payload

    return None


def is_control_message(line: str) -> bool:
    """True for JSON server control messages (legacy) or stream command lines."""
    if parse_stream_command(line) is not None:
        return True
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


def _quat_dot(a: dict[str, float], b: dict[str, float]) -> float:
    return (
        a["w"] * b["w"]
        + a["x"] * b["x"]
        + a["y"] * b["y"]
        + a["z"] * b["z"]
    )


def _quat_mult(a: dict[str, float], b: dict[str, float]) -> dict[str, float]:
    return {
        "w": a["w"] * b["w"] - a["x"] * b["x"] - a["y"] * b["y"] - a["z"] * b["z"],
        "x": a["w"] * b["x"] + a["x"] * b["w"] + a["y"] * b["z"] - a["z"] * b["y"],
        "y": a["w"] * b["y"] - a["x"] * b["z"] + a["y"] * b["w"] + a["z"] * b["x"],
        "z": a["w"] * b["z"] + a["x"] * b["y"] - a["y"] * b["x"] + a["z"] * b["w"],
    }


def imu_quat_to_body_frame(
    imu_q: dict[str, float],
    imu_to_body: dict[str, float],
) -> dict[str, float]:
    """
    Collar body attitude from raw IMU quaternion and imu_to_body mount.

    Matches fusion.c fusion_measured_body_attitude: q_body = q_imu * inv(mount).
    """
    q_imu = _quat_normalize(imu_q)
    mount = _quat_normalize(imu_to_body)
    if (
        abs(mount["w"] - 1.0) < 1e-6
        and abs(mount["x"]) < 1e-6
        and abs(mount["y"]) < 1e-6
        and abs(mount["z"]) < 1e-6
    ):
        return q_imu
    return _quat_normalize(_quat_mult(q_imu, _quat_conj(mount)))


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
    gap_us = hi.ts_us - lo.ts_us
    if gap_us > MAX_QUAT_SLERP_GAP_US:
        # Sparse IMU: hold previous orientation, jump at the new sample time.
        if ts_us >= hi.ts_us:
            return dict(hi.value)
        return dict(lo.value)
    alpha = (ts_us - lo.ts_us) / gap_us
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
    *,
    max_pixels_per_frame: int = 40,
    min_quality: int = 25,
) -> dict[str, int]:
    dx = dy = 0
    quality = 0
    found = False
    for sample in samples:
        if t_lo_us < sample.ts_us <= t_hi_us:
            val = sample.value
            sdx = int(val["dx"])
            sdy = int(val["dy"])
            sq = int(val.get("quality", 255))
            if sq < min_quality:
                continue
            if abs(sdx) > max_pixels_per_frame or abs(sdy) > max_pixels_per_frame:
                # One corrupt PMW3901 frame — drop the whole tick interval.
                return dict(DEFAULT_FLOW)
            dx += sdx
            dy += sdy
            quality = max(quality, sq)
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


def gyro_from_quat_window(
    quat_samples: list[tuple[float, dict[str, float]]],
    t_ms: float,
    *,
    window_s: float = 0.2,
) -> dict[str, float]:
    """
    Angular rate at ``t_ms`` from quaternion change over ``window_s``.

    Uses a wide baseline so gyro estimates are not dominated by
  per-sample quaternion noise (game-rotation vectors can jitter at 100 Hz
    while the true orientation changes slowly).
    """
    if not quat_samples or window_s <= 0.0:
        return dict(DEFAULT_ACCEL)

    t_lo = t_ms - window_s * 1000.0

    def sample_at(target: float) -> tuple[dict[str, float], float] | None:
        if target <= quat_samples[0][0]:
            return quat_samples[0][1], quat_samples[0][0]
        if target >= quat_samples[-1][0]:
            return quat_samples[-1][1], quat_samples[-1][0]
        lo = quat_samples[0]
        hi = quat_samples[-1]
        for i in range(len(quat_samples) - 1):
            a = quat_samples[i]
            b = quat_samples[i + 1]
            if a[0] <= target <= b[0]:
                lo, hi = a, b
                break
        if math.isclose(lo[0], hi[0]):
            return lo[1], lo[0]
        alpha = (target - lo[0]) / (hi[0] - lo[0])
        q0 = _quat_normalize(lo[1])
        q1 = _quat_normalize(hi[1])
        dot = _quat_dot(q0, q1)
        if dot < 0.0:
            q1 = {k: -v for k, v in q1.items()}
            dot = -dot
        if dot > 0.9995:
            blended = {
                "w": _lerp(q0["w"], q1["w"], alpha),
                "x": _lerp(q0["x"], q1["x"], alpha),
                "y": _lerp(q0["y"], q1["y"], alpha),
                "z": _lerp(q0["z"], q1["z"], alpha),
            }
            return _quat_normalize(blended), target
        theta_0 = math.acos(max(-1.0, min(1.0, dot)))
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * alpha
        s0 = math.sin(theta_0 - theta) / sin_theta_0
        s1 = math.sin(theta) / sin_theta_0
        blended = _quat_normalize({
            "w": s0 * q0["w"] + s1 * q1["w"],
            "x": s0 * q0["x"] + s1 * q1["x"],
            "y": s0 * q0["y"] + s1 * q1["y"],
            "z": s0 * q0["z"] + s1 * q1["z"],
        })
        return blended, target

    curr = sample_at(t_ms)
    prev = sample_at(t_lo)
    if curr is None or prev is None:
        return dict(DEFAULT_ACCEL)
    q_curr, t_curr = curr
    q_prev, t_prev = prev
    dt_s = (t_curr - t_prev) / 1000.0
    if dt_s < window_s * 0.5:
        return dict(DEFAULT_ACCEL)
    return gyro_from_quat_pair(q_prev, q_curr, dt_s)


@dataclass
class LatencyEstimator:
    """Estimate minimum safe buffer latency from observed sensor update intervals."""

    min_latency_us: int = 50_000
    max_latency_us: int = 2_000_000
    margin_periods: float = 1.5
    window_size: int = 30
    min_samples: int = 3

    _last_ts_us: dict[int, int] = field(default_factory=dict)
    _intervals_us: dict[int, list[int]] = field(default_factory=lambda: defaultdict(list))

    def reset(self) -> None:
        self._last_ts_us.clear()
        self._intervals_us.clear()

    def observe(self, sensor: int, ts_us: int) -> None:
        prev = self._last_ts_us.get(sensor)
        if prev is not None:
            delta = ts_us - prev
            if 1_000 < delta < 10_000_000:
                bucket = self._intervals_us[sensor]
                bucket.append(delta)
                if len(bucket) > self.window_size:
                    del bucket[0]
        self._last_ts_us[sensor] = ts_us

    def latency_us(self, output_hz: float, fallback_us: int) -> int:
        periods: list[float] = []
        for intervals in self._intervals_us.values():
            if len(intervals) >= self.min_samples:
                periods.append(float(statistics.median(intervals)))

        if not periods:
            return max(self.min_latency_us, min(fallback_us, self.max_latency_us))

        slowest_us = max(periods)
        output_period_us = 1_000_000.0 / max(output_hz, 1.0)
        estimated = int(slowest_us * self.margin_periods + output_period_us)
        return max(self.min_latency_us, min(estimated, self.max_latency_us))

    def sensor_periods_ms(self) -> dict[str, float]:
        labels = {
            SENSOR_ACCEL: "accel",
            SENSOR_QUAT: "quat",
            SENSOR_FLOW: "flow",
            SENSOR_RADAR: "radar",
        }
        out: dict[str, float] = {}
        for sensor, intervals in self._intervals_us.items():
            if len(intervals) >= self.min_samples:
                out[labels.get(sensor, str(sensor))] = statistics.median(intervals) / 1000.0
        return out


@dataclass
class SensorStreamBuffer:
    """Buffers heterogeneous sensor samples and emits regular fused ticks."""

    fixed_latency_us: int | None = None
    min_latency_us: int = 50_000
    max_latency_us: int = 2_000_000
    max_history_us: int = 30_000_000
    max_ticks_per_ingest: int = 64
    output_hz: float = 100.0
    flow_max_pixels_per_frame: int = 40
    flow_min_quality: int = 25
    on_tick: Callable[[dict[str, Any]], None] | None = None
    on_latency_change: Callable[[int, dict[str, float]], None] | None = None

    accel: list[TimedSample] = field(default_factory=list)
    quat: list[TimedSample] = field(default_factory=list)
    flow: list[TimedSample] = field(default_factory=list)
    radar: list[TimedSample] = field(default_factory=list)

    _latest_ts_us: int = 0
    _next_emit_us: int | None = None
    _prev_quat: dict[str, float] | None = None
    _prev_quat_ts_us: int | None = None
    _gyro_smooth: dict[str, float] | None = None
    gyro_smooth_alpha: float = 0.3
    _latency_estimator: LatencyEstimator = field(default_factory=LatencyEstimator)
    _current_latency_us: int = 50_000
    _lock: threading.Lock = field(default_factory=threading.Lock)

    # Backward-compatible alias for fixed latency configuration.
    @property
    def latency_us(self) -> int:
        return self._current_latency_us

    @latency_us.setter
    def latency_us(self, value: int) -> None:
        self.fixed_latency_us = value
        self._current_latency_us = value

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
            self._gyro_smooth = None
            self._latency_estimator.reset()
            self._latency_estimator.min_latency_us = self.min_latency_us
            self._latency_estimator.max_latency_us = self.max_latency_us
            self._current_latency_us = self.min_latency_us

    def _effective_latency_us_locked(self) -> int:
        if self.fixed_latency_us is not None:
            return self.fixed_latency_us
        fallback = self._current_latency_us or self.min_latency_us
        return self._latency_estimator.latency_us(self.output_hz, fallback)

    def _update_latency_locked(self) -> None:
        if self.fixed_latency_us is not None:
            return
        new_latency = self._effective_latency_us_locked()
        if abs(new_latency - self._current_latency_us) < 5_000:
            return
        self._current_latency_us = new_latency
        if self.on_latency_change is not None:
            self.on_latency_change(new_latency, self._latency_estimator.sensor_periods_ms())

    def stream_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "latency_us": self._current_latency_us,
                "latency_ms": round(self._current_latency_us / 1000.0, 1),
                "adaptive": self.fixed_latency_us is None,
                "sensor_period_ms": self._latency_estimator.sensor_periods_ms(),
                "output_hz": self.output_hz,
                "latest_ts_us": self._latest_ts_us,
            }

    def ingest(self, sensor: int, ts_us: int, data: dict[str, Any]) -> None:
        with self._lock:
            self._ingest_locked(sensor, ts_us, data)
            ticks = self._collect_drain_locked()
        self._emit_ticks(ticks)

    def ingest_sequence(
        self,
        samples: list[tuple[int, int, dict[str, Any]]],
    ) -> None:
        """Ingest an unpacked batch in order, then emit all ready ticks."""
        if not samples:
            return
        ticks: list[dict[str, Any]] = []
        with self._lock:
            for sensor, ts_us, data in samples:
                self._ingest_locked(sensor, ts_us, data)
            while True:
                batch = self._collect_drain_locked()
                if not batch:
                    break
                ticks.extend(batch)
        self._emit_ticks(ticks)

    def _ingest_locked(self, sensor: int, ts_us: int, data: dict[str, Any]) -> None:
            if sensor == SENSOR_ACCEL:
                self.accel.append(TimedSample(ts_us, {
                    "x": float(data["x"]), "y": float(data["y"]), "z": float(data["z"]),
                }))
            elif sensor == SENSOR_QUAT:
                q = {
                    "w": float(data["w"]), "x": float(data["x"]),
                    "y": float(data["y"]), "z": float(data["z"]),
                }
                q = _quat_normalize(q)
                if self.quat:
                    last = self.quat[-1]
                    if _quat_dot(q, last.value) >= QUAT_DEDUP_DOT_THRESHOLD:
                        self.quat[-1] = TimedSample(ts_us, last.value)
                    else:
                        self.quat.append(TimedSample(ts_us, q))
                else:
                    self.quat.append(TimedSample(ts_us, q))
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

            self._latency_estimator.observe(sensor, ts_us)
            self._update_latency_locked()

            self._sort_channel_if_needed(self.accel)
            self._sort_channel_if_needed(self.quat)
            self._sort_channel_if_needed(self.flow)
            self._sort_channel_if_needed(self.radar)

            self._latest_ts_us = max(self._latest_ts_us, ts_us)
            self._prune_locked()

    def _sort_channel_if_needed(self, channel: list[TimedSample]) -> None:
        if len(channel) >= 2 and channel[-1].ts_us < channel[-2].ts_us:
            channel.sort(key=lambda s: s.ts_us)

    def _prune_locked(self) -> None:
        cutoff = self._latest_ts_us - self.max_history_us
        if cutoff <= 0:
            return
        for channel in (self.accel, self.quat, self.flow, self.radar):
            if not channel:
                continue
            idx = 0
            while idx < len(channel) and channel[idx].ts_us < cutoff:
                idx += 1
            if idx:
                del channel[:idx]

    def _emit_ticks(self, ticks: list[dict[str, Any]]) -> None:
        if self.on_tick is None:
            return
        for tick in ticks:
            self.on_tick(tick)

    def _collect_drain_locked(self) -> list[dict[str, Any]]:
        latency_us = self._effective_latency_us_locked()
        dt_us = int(1_000_000 / self.output_hz)
        ready_until = self._latest_ts_us - latency_us
        if ready_until <= 0:
            return []

        if self._next_emit_us is None:
            if not self.quat and not self.accel:
                return []
            earliest = min(
                (self.accel[0].ts_us if self.accel else 2**62),
                (self.quat[0].ts_us if self.quat else 2**62),
            )
            self._next_emit_us = earliest + latency_us

        ticks: list[dict[str, Any]] = []
        prev_flow_lo = self._next_emit_us - dt_us
        emitted = 0
        while self._next_emit_us <= ready_until and emitted < self.max_ticks_per_ingest:
            tick = self._bundle_at_locked(self._next_emit_us, prev_flow_lo)
            if tick is not None:
                ticks.append(tick)
            prev_flow_lo = self._next_emit_us
            self._next_emit_us += dt_us
            emitted += 1
        return ticks

    def _bundle_at_locked(self, ts_us: int, flow_lo_us: int) -> dict[str, Any] | None:
        quat = _interp_quat_channel(self.quat, ts_us, DEFAULT_QUAT)
        accel = _interp_vec3_channel(self.accel, ts_us, DEFAULT_ACCEL)
        flow = _flow_in_interval(
            self.flow, flow_lo_us, ts_us,
            max_pixels_per_frame=self.flow_max_pixels_per_frame,
            min_quality=self.flow_min_quality,
        )
        range_mm = int(round(_interp_scalar_channel(
            self.radar, ts_us, float(DEFAULT_RANGE_MM),
        )))

        if self._prev_quat is not None and self._prev_quat_ts_us is not None:
            quat_series = [(s.ts_us / 1000.0, s.value) for s in self.quat]
            raw_gyro = gyro_from_quat_window(
                quat_series,
                ts_us / 1000.0,
                window_s=QUAT_GYRO_WINDOW_S,
            )
            mag = math.sqrt(
                raw_gyro["x"] ** 2 + raw_gyro["y"] ** 2 + raw_gyro["z"] ** 2
            )
            if mag > MAX_QUAT_GYRO_RAD_S and mag > 1e-6:
                scale = MAX_QUAT_GYRO_RAD_S / mag
                raw_gyro = {k: raw_gyro[k] * scale for k in raw_gyro}
            if self._gyro_smooth is None:
                self._gyro_smooth = dict(raw_gyro)
            else:
                alpha = self.gyro_smooth_alpha
                self._gyro_smooth = {
                    k: self._gyro_smooth[k] + alpha * (raw_gyro[k] - self._gyro_smooth[k])
                    for k in raw_gyro
                }
            gyro = dict(self._gyro_smooth)
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
        ticks: list[dict[str, Any]] = []
        with self._lock:
            saved_fixed = self.fixed_latency_us
            self.fixed_latency_us = 0
            self._current_latency_us = 0
            while True:
                batch = self._collect_drain_locked()
                if not batch:
                    break
                ticks.extend(batch)
            self.fixed_latency_us = saved_fixed
            if self.fixed_latency_us is None:
                self._update_latency_locked()
            else:
                self._current_latency_us = self.fixed_latency_us
        self._emit_ticks(ticks)
