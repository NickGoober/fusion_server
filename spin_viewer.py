#!/usr/bin/env python3
"""
2D replay viewer for collar spin captures.

Left panel: bar rotating from game-rotation quaternion (top-down XY).
Right panel: phasor for gravity (wire type 1, new firmware) or linear accel (legacy).
When gravity is present, a second arrow shows linear acceleration (gravity removed):
  measured as wire type 4 accelerometer minus gravity, or legacy wire type 1.

Requires: pip install matplotlib numpy

Examples:
  python spin_viewer.py motorSpinFinal.jsonl
  python spin_viewer.py capture1.jsonl --accel-axes xy --speed 2
  python spin_viewer.py gravitySpin.jsonl --accel-axes zy --accel-limit 12
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import animation
from matplotlib.widgets import Button, CheckButtons, RadioButtons, Slider

from device_protocol import unpack_collar_wire_line
from lever_arm_comp import IMU_LEVER_ARM_M, kinematic_accel
from sensor_stream import SENSOR_ACCEL, SENSOR_QUAT, gyro_from_quat_pair

# Collar wire types (see device_protocol.py).
WIRE_QUAT = 0
WIRE_VEC3 = 1
WIRE_ACCEL = 4

GRAVITY_STREAM_MAG_THRESHOLD = 5.0
LEGACY_LINEAR_STREAM_MAG_THRESHOLD = 2.5

# Body frame: +X right, +Y forward, +Z up (looking along the bar from the front).
BAR_HALF_M = 0.12
SPIN_LIMIT_M = 0.35
ACCEL_LIMIT_MPS2 = 0.5  # fixed phasor axis range (m/s²)

# Runtime phasor filters (id, short UI label).
PHASOR_FILTERS: tuple[tuple[str, str], ...] = (
    ("none", "None"),
    ("ema", "EMA"),
    ("kalman", "Kalman"),
    ("moving_avg", "Mov avg"),
    ("median", "Median"),
    ("polar_ema", "Polar EMA"),
    ("mag_ema", "Mag EMA"),
)


@dataclass
class Frame:
    ts_us: int
    quat: dict[str, float]
    gravity: dict[str, float] | None = None
    linear_accel: dict[str, float] | None = None
    omega_rad_s: float | None = None
    omega_vec: dict[str, float] | None = None
    linear_source: str = "none"


@dataclass
class KfConfig:
    enabled: bool = True
    accel_q: float = 0.03
    accel_r: float = 0.2
    quat_q: float = 0.002
    quat_r: float = 0.08


class Kalman1D:
    """Scalar random-walk Kalman filter (light smoothing)."""

    def __init__(self, process_var: float, measurement_var: float) -> None:
        self.q = process_var
        self.r = measurement_var
        self.x: float | None = None
        self.p = 1.0

    def update(self, z: float) -> float:
        if self.x is None:
            self.x = z
            self.p = self.r
            return z
        p_pred = self.p + self.q
        k = p_pred / (p_pred + self.r)
        self.x = self.x + k * (z - self.x)
        self.p = (1.0 - k) * p_pred
        return self.x


def _normalize_quat(q: dict[str, float]) -> dict[str, float]:
    n = math.sqrt(q["w"] ** 2 + q["x"] ** 2 + q["y"] ** 2 + q["z"] ** 2)
    if n < 1e-12:
        return {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
    return {k: q[k] / n for k in ("w", "x", "y", "z")}


def _quat_same_hemisphere(q: dict[str, float], ref: dict[str, float]) -> dict[str, float]:
    dot = q["w"] * ref["w"] + q["x"] * ref["x"] + q["y"] * ref["y"] + q["z"] * ref["z"]
    if dot < 0.0:
        return {k: -q[k] for k in q}
    return q


def _filter_accel_stream(
    accels: list[tuple[int, dict[str, float]]],
    q: float,
    r: float,
) -> list[tuple[int, dict[str, float]]]:
    kf = {axis: Kalman1D(q, r) for axis in ("x", "y", "z")}
    out: list[tuple[int, dict[str, float]]] = []
    for ts, data in accels:
        out.append((ts, {axis: kf[axis].update(data[axis]) for axis in ("x", "y", "z")}))
    return out


def _filter_quat_stream(
    quats: list[tuple[int, dict[str, float]]],
    q: float,
    r: float,
) -> list[tuple[int, dict[str, float]]]:
    kf = {key: Kalman1D(q, r) for key in ("w", "x", "y", "z")}
    out: list[tuple[int, dict[str, float]]] = []
    prev: dict[str, float] | None = None
    for ts, data in quats:
        sample = dict(data)
        if prev is not None:
            sample = _quat_same_hemisphere(sample, prev)
        filtered = {key: kf[key].update(sample[key]) for key in ("w", "x", "y", "z")}
        normed = _normalize_quat(filtered)
        out.append((ts, normed))
        prev = normed
    return out


def _quat_rotate(q: dict[str, float], x: float, y: float, z: float) -> tuple[float, float, float]:
    """Rotate body-frame vector by quaternion (w, x, y, z)."""
    w, qx, qy, qz = q["w"], q["x"], q["y"], q["z"]
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + w * tx + (qy * tz - qz * ty),
        y + w * ty + (qz * tx - qx * tz),
        z + w * tz + (qx * ty - qy * tx),
    )


def _vec3_magnitude(vec: dict[str, float]) -> float:
    return math.sqrt(vec["x"] ** 2 + vec["y"] ** 2 + vec["z"] ** 2)


def _subtract_vec3(a: dict[str, float], b: dict[str, float]) -> dict[str, float]:
    return {"x": a["x"] - b["x"], "y": a["y"] - b["y"], "z": a["z"] - b["z"]}


def _median(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def _detect_wire_vec3_mode(samples: list[tuple[int, dict[str, float]]]) -> str:
    """Return 'gravity', 'linear', or 'unknown' for collar wire type 1."""
    if not samples:
        return "unknown"
    mags = [_vec3_magnitude(data) for _, data in samples]
    med = _median(mags)
    if med >= GRAVITY_STREAM_MAG_THRESHOLD:
        return "gravity"
    if med <= LEGACY_LINEAR_STREAM_MAG_THRESHOLD:
        return "linear"
    return "unknown"


def _load_wire_vec3_batches(path: Path) -> tuple[
    list[tuple[int, dict[str, float]]],
    list[tuple[int, dict[str, float]]],
    list[tuple[int, dict[str, float]]],
    list[tuple[int, dict[str, float]]],
]:
    """Load collar wire quats (0), type-1 vec3, accelerometer (4), and optional derived linear."""
    quats: list[tuple[int, dict[str, float]]] = []
    wire1: list[tuple[int, dict[str, float]]] = []
    wire_accel: list[tuple[int, dict[str, float]]] = []
    wire_linear: list[tuple[int, dict[str, float]]] = []

    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("{"):
                continue
            try:
                batch = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(batch, list):
                continue
            rows: list[list[Any]]
            if batch and isinstance(batch[0], list):
                rows = batch
            elif len(batch) >= 3 and len(batch) % 3 == 0:
                rows = [batch[offset: offset + 3] for offset in range(0, len(batch), 3)]
            elif len(batch) == 3:
                rows = [batch]
            else:
                continue

            for row in rows:
                if not isinstance(row, list) or len(row) < 3:
                    continue
                try:
                    sensor = int(row[0])
                    ts_us = int(row[1])
                    payload = row[2]
                except (TypeError, ValueError):
                    continue
                if not isinstance(payload, list):
                    continue
                if sensor == WIRE_QUAT and len(payload) >= 4:
                    quats.append((
                        ts_us,
                        {
                            "x": float(payload[0]),
                            "y": float(payload[1]),
                            "z": float(payload[2]),
                            "w": float(payload[3]),
                        },
                    ))
                elif sensor == WIRE_VEC3 and len(payload) >= 6:
                    wire1.append((
                        ts_us,
                        {
                            "x": float(payload[0]),
                            "y": float(payload[1]),
                            "z": float(payload[2]),
                        },
                    ))
                    wire_linear.append((
                        ts_us,
                        {
                            "x": float(payload[3]),
                            "y": float(payload[4]),
                            "z": float(payload[5]),
                        },
                    ))
                elif sensor == WIRE_VEC3 and len(payload) == 3:
                    wire1.append((
                        ts_us,
                        {
                            "x": float(payload[0]),
                            "y": float(payload[1]),
                            "z": float(payload[2]),
                        },
                    ))
                elif sensor == WIRE_ACCEL and len(payload) == 3:
                    wire_accel.append((
                        ts_us,
                        {
                            "x": float(payload[0]),
                            "y": float(payload[1]),
                            "z": float(payload[2]),
                        },
                    ))

    quats.sort(key=lambda item: item[0])
    wire1.sort(key=lambda item: item[0])
    wire_accel.sort(key=lambda item: item[0])
    wire_linear.sort(key=lambda item: item[0])
    return quats, wire1, wire_accel, wire_linear


def _nearest_sample(
    samples: list[tuple[int, dict[str, float]]],
    ts_us: int,
    *,
    max_delta_us: int = 50_000,
) -> dict[str, float] | None:
    if not samples:
        return None
    idx = 0
    while idx + 1 < len(samples) and samples[idx + 1][0] <= ts_us:
        idx += 1
    ts, data = samples[idx]
    if abs(ts - ts_us) > max_delta_us:
        return None
    return data


def _parse_accel_axes(spec: str) -> tuple[str, str]:
    """Two distinct body axes: first = phasor horizontal, second = vertical."""
    s = spec.lower().strip()
    if len(s) != 2 or s[0] not in "xyz" or s[1] not in "xyz" or s[0] == s[1]:
        raise argparse.ArgumentTypeError(
            "accel-axes must be two distinct letters from xyz (e.g. zy, xy, xz)"
        )
    return s[0], s[1]


def _strength_to_alpha(strength: float) -> float:
    """Map smooth slider 0..100 → EMA alpha (lower alpha = smoother)."""
    t = max(0.0, min(100.0, strength)) / 100.0
    return 0.35 - t * 0.32  # 0.35 light … 0.03 heavy


def _strength_to_window(strength: float) -> int:
    t = max(0.0, min(100.0, strength)) / 100.0
    return max(3, int(3 + t * 47))  # 3 … 50 samples


def _strength_to_kalman(strength: float) -> tuple[float, float]:
    t = max(0.0, min(100.0, strength)) / 100.0
    q = 0.08 - t * 0.075
    r = 0.55 - t * 0.45
    return max(1e-4, q), max(0.05, r)


def _ema_angle(prev: float, measured: float, alpha: float) -> float:
    adj = measured
    while adj - prev > math.pi:
        adj -= 2.0 * math.pi
    while adj - prev < -math.pi:
        adj += 2.0 * math.pi
    return prev + alpha * (adj - prev)


def _smooth_phasor_series(
    h_raw: list[float | None],
    v_raw: list[float | None],
    *,
    mode: str,
    strength: float,
) -> tuple[list[float | None], list[float | None]]:
    n = len(h_raw)
    if mode == "none":
        return list(h_raw), list(v_raw)

    h_out: list[float | None] = [None] * n
    v_out: list[float | None] = [None] * n
    alpha = _strength_to_alpha(strength)
    window = _strength_to_window(strength)
    kf_q, kf_r = _strength_to_kalman(strength)

    if mode == "ema":
        sh = sv = None
        for i in range(n):
            h, v = h_raw[i], v_raw[i]
            if h is None or v is None:
                continue
            sh = h if sh is None else sh + alpha * (h - sh)
            sv = v if sv is None else sv + alpha * (v - sv)
            h_out[i], v_out[i] = sh, sv
        return h_out, v_out

    if mode == "kalman":
        kf_h = Kalman1D(kf_q, kf_r)
        kf_v = Kalman1D(kf_q, kf_r)
        for i in range(n):
            h, v = h_raw[i], v_raw[i]
            if h is None or v is None:
                continue
            h_out[i] = kf_h.update(h)
            v_out[i] = kf_v.update(v)
        return h_out, v_out

    if mode == "moving_avg":
        h_buf: deque[float] = deque(maxlen=window)
        v_buf: deque[float] = deque(maxlen=window)
        for i in range(n):
            h, v = h_raw[i], v_raw[i]
            if h is None or v is None:
                continue
            h_buf.append(h)
            v_buf.append(v)
            h_out[i] = sum(h_buf) / len(h_buf)
            v_out[i] = sum(v_buf) / len(v_buf)
        return h_out, v_out

    if mode == "median":
        h_buf = deque(maxlen=window)
        v_buf = deque(maxlen=window)
        for i in range(n):
            h, v = h_raw[i], v_raw[i]
            if h is None or v is None:
                continue
            h_buf.append(h)
            v_buf.append(v)
            h_sorted = sorted(h_buf)
            v_sorted = sorted(v_buf)
            h_out[i] = h_sorted[len(h_sorted) // 2]
            v_out[i] = v_sorted[len(v_sorted) // 2]
        return h_out, v_out

    if mode == "polar_ema":
        sr = sa = None
        for i in range(n):
            h, v = h_raw[i], v_raw[i]
            if h is None or v is None:
                continue
            r = math.hypot(h, v)
            a = math.atan2(v, h)
            sr = r if sr is None else sr + alpha * (r - sr)
            sa = a if sa is None else _ema_angle(sa, a, alpha)
            h_out[i] = sr * math.cos(sa)
            v_out[i] = sr * math.sin(sa)
        return h_out, v_out

    if mode == "mag_ema":
        sr = None
        angle_alpha = min(0.35, alpha * 2.5)
        sa = None
        for i in range(n):
            h, v = h_raw[i], v_raw[i]
            if h is None or v is None:
                continue
            r = math.hypot(h, v)
            a = math.atan2(v, h)
            sr = r if sr is None else sr + alpha * (r - sr)
            sa = a if sa is None else _ema_angle(sa, a, angle_alpha)
            h_out[i] = sr * math.cos(sa)
            v_out[i] = sr * math.sin(sa)
        return h_out, v_out

    raise ValueError(f"unknown phasor filter {mode!r}")


def _extract_phasor_series(
    frames: list[Frame],
    h_axis: str,
    v_axis: str,
    *,
    vector: str,
) -> tuple[list[float | None], list[float | None]]:
    h_raw: list[float | None] = []
    v_raw: list[float | None] = []
    for frame in frames:
        data = frame.gravity if vector == "gravity" else frame.linear_accel
        if data is None:
            h_raw.append(None)
            v_raw.append(None)
        else:
            h_raw.append(data[h_axis])
            v_raw.append(data[v_axis])
    return h_raw, v_raw


def _project(vec: tuple[float, float, float], view: str) -> tuple[float, float]:
    x, y, z = vec
    if view == "xy":
        return x, y
    if view == "xz":
        return x, z
    if view == "yz":
        return y, z
    raise ValueError(f"unknown view {view!r}")


def _bar_polygon_2d(q: dict[str, float], view: str, half_m: float) -> list[tuple[float, float]]:
    """Bar along body +X, thickness along body +Y."""
    thickness = half_m * 0.18
    corners_body = [
        (half_m, thickness),
        (half_m, -thickness),
        (-half_m, -thickness),
        (-half_m, thickness),
    ]
    out: list[tuple[float, float]] = []
    for bx, by, bz in ((x, y, 0.0) for x, y in corners_body):
        wx, wy, wz = _quat_rotate(q, bx, by, bz)
        out.append(_project((wx, wy, wz), view))
    return out


def _load_frames(path: Path, *, kf: KfConfig | None = None) -> tuple[list[Frame], str]:
    quats, wire1, wire_accel, wire_linear = _load_wire_vec3_batches(path)

    if not quats:
        # Fall back to generic unpack for non-collar captures.
        quats = []
        wire1 = []
        wire_linear = []
        with path.open(encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line or line.startswith("{"):
                    continue
                for sample in unpack_collar_wire_line(line):
                    if sample.sensor == SENSOR_QUAT:
                        quats.append((sample.ts_us, sample.data))
                    elif sample.sensor == SENSOR_ACCEL:
                        wire1.append((sample.ts_us, sample.data))
        quats.sort(key=lambda item: item[0])
        wire1.sort(key=lambda item: item[0])

    if not quats:
        raise SystemExit(f"No quaternions in {path}")

    wire1_mode = _detect_wire_vec3_mode(wire1)
    has_gravity_stream = wire1_mode == "gravity"
    has_legacy_linear = wire1_mode == "linear"
    has_specific_accel = bool(wire_accel)
    has_file_linear = bool(wire_linear)

    if kf and kf.enabled:
        quats = _filter_quat_stream(quats, kf.quat_q, kf.quat_r)
        if wire1:
            wire1 = _filter_accel_stream(wire1, kf.accel_q, kf.accel_r)
        if wire_accel:
            wire_accel = _filter_accel_stream(wire_accel, kf.accel_q, kf.accel_r)
        if wire_linear:
            wire_linear = _filter_accel_stream(wire_linear, kf.accel_q, kf.accel_r)

    frames: list[Frame] = []
    prev_omega_vec: dict[str, float] | None = None
    prev_ts: int | None = None

    for i, (ts, quat) in enumerate(quats):
        gravity = _nearest_sample(wire1, ts) if has_gravity_stream else None
        legacy_linear = _nearest_sample(wire1, ts) if has_legacy_linear else None
        file_linear = _nearest_sample(wire_linear, ts) if has_file_linear else None
        specific = _nearest_sample(wire_accel, ts)

        omega = None
        omega_vec: dict[str, float] | None = None
        if i > 0:
            prev_ts_q, prev_q = quats[i - 1]
            dt = (ts - prev_ts_q) / 1_000_000.0
            if 0.0 < dt < 0.1:
                omega_vec = gyro_from_quat_pair(prev_q, quat, dt)
                omega = math.sqrt(
                    omega_vec["x"] ** 2 + omega_vec["y"] ** 2 + omega_vec["z"] ** 2,
                )

        linear_accel: dict[str, float] | None = None
        linear_source = "none"
        if file_linear is not None:
            linear_accel = file_linear
            linear_source = "file"
        elif legacy_linear is not None:
            linear_accel = legacy_linear
            linear_source = "legacy"
        elif specific is not None and gravity is not None:
            linear_accel = _subtract_vec3(specific, gravity)
            linear_source = "computed"
        elif specific is not None:
            linear_accel = specific
            linear_source = "accel_only"
        elif gravity is not None and omega_vec is not None and prev_omega_vec is not None and prev_ts is not None:
            dt = (ts - prev_ts) / 1_000_000.0
            if dt > 1e-6:
                omega_dot = (
                    (omega_vec["x"] - prev_omega_vec["x"]) / dt,
                    (omega_vec["y"] - prev_omega_vec["y"]) / dt,
                    (omega_vec["z"] - prev_omega_vec["z"]) / dt,
                )
                arm = (IMU_LEVER_ARM_M["x"], IMU_LEVER_ARM_M["y"], IMU_LEVER_ARM_M["z"])
                ox, oy, oz = kinematic_accel(
                    (omega_vec["x"], omega_vec["y"], omega_vec["z"]),
                    omega_dot,
                    arm,
                )
                linear_accel = {"x": ox, "y": oy, "z": oz}
                linear_source = "kinematic"

        frames.append(Frame(
            ts_us=ts,
            quat=quat,
            gravity=gravity,
            linear_accel=linear_accel,
            omega_rad_s=omega,
            omega_vec=omega_vec,
            linear_source=linear_source,
        ))
        prev_omega_vec = omega_vec
        prev_ts = ts

    phasor_mode = "gravity" if has_gravity_stream else "linear"
    return frames, phasor_mode


def _spin_axis_line(axis: str) -> tuple[tuple[float, float], tuple[float, float]]:
    """Spin axis through origin in the XY top-down view."""
    length = SPIN_LIMIT_M * 0.9
    if axis == "x":
        v = (1.0, 0.0, 0.0)
    elif axis == "y":
        v = (0.0, 1.0, 0.0)
    elif axis == "z":
        v = (0.0, 0.0, 1.0)
    else:
        raise ValueError(axis)
    px, py = _project(v, "xy")
    n = math.hypot(px, py) or 1.0
    px /= n
    py /= n
    return ((-px * length, -py * length), (px * length, py * length))


def run_viewer(
    frames: list[Frame],
    *,
    phasor_mode: str,
    spin_axis: str,
    speed: float,
    title: str,
    accel_limit: float,
    linear_limit: float,
    accel_axes: tuple[str, str],
) -> None:
    h_axis, v_axis = accel_axes
    plane_tag = f"{h_axis}{v_axis}"
    t0_us = frames[0].ts_us
    t_end_s = (frames[-1].ts_us - t0_us) / 1_000_000.0

    fig, (ax_spin, ax_accel) = plt.subplots(1, 2, figsize=(14, 7))
    plt.subplots_adjust(bottom=0.38, wspace=0.28, right=0.98)
    fig.suptitle(title, fontsize=12)

    for ax in (ax_spin, ax_accel):
        ax.set_aspect("equal", adjustable="box")
        ax.axhline(0, color="#444", linewidth=0.6)
        ax.axvline(0, color="#444", linewidth=0.6)
        ax.grid(True, alpha=0.25)

    ax_spin.set_xlim(-SPIN_LIMIT_M, SPIN_LIMIT_M)
    ax_spin.set_ylim(-SPIN_LIMIT_M, SPIN_LIMIT_M)
    ax_spin.set_xlabel("X right (m)")
    ax_spin.set_ylabel("Y forward (m)")
    ax_spin.set_title("Collar rotation (XY)")

    ax_accel.set_xlim(-accel_limit, accel_limit)
    ax_accel.set_ylim(-accel_limit, accel_limit)
    primary_label = "Gravity" if phasor_mode == "gravity" else "Linear accel"
    ax_accel.set_xlabel(f"{primary_label} a_{h_axis} body (m/s²)")
    ax_accel.set_ylabel(f"{primary_label} a_{v_axis} body (m/s²)")
    ax_accel.set_title(f"{primary_label} {plane_tag.upper()} phasor (body frame)")

    axis_line = _spin_axis_line(spin_axis)
    ax_spin.plot(
        [axis_line[0][0], axis_line[1][0]],
        [axis_line[0][1], axis_line[1][1]],
        color="#f59e0b",
        linewidth=2,
        linestyle="--",
        label=f"spin axis ({spin_axis.upper()})",
    )

    bar_patch = mpatches.Polygon(
        [[0, 0]], closed=True, facecolor="#4cc9f0", edgecolor="#1e3a4f", alpha=0.85,
    )
    ax_spin.add_patch(bar_patch)
    ax_spin.plot([0], [0], "o", color="#ef4444", markersize=6, label="IMU center")
    ax_spin.legend(loc="upper right", fontsize=8)

    quiver_kw = dict(angles="xy", scale_units="xy", scale=1.0)
    accel_arrow = ax_accel.quiver(
        0, 0, 0, 0, **quiver_kw,
        color="#22c55e", width=0.012, headwidth=4, headlength=6,
        label=f"{primary_label} {plane_tag}",
    )
    accel_h_arrow = ax_accel.quiver(
        0, 0, 0, 0, **quiver_kw,
        color="#ef4444", width=0.009, headwidth=4, headlength=5,
        label=f"{primary_label} a_{h_axis}",
    )
    accel_v_arrow = ax_accel.quiver(
        0, 0, 0, 0, **quiver_kw,
        color="#3b82f6", width=0.009, headwidth=4, headlength=5,
        label=f"{primary_label} a_{v_axis}",
    )
    linear_arrow = ax_accel.quiver(
        0, 0, 0, 0, **quiver_kw,
        color="#f97316", width=0.011, headwidth=4, headlength=6,
        label=f"Linear (no g) {plane_tag}",
    )
    ax_accel.legend(loc="upper right", fontsize=8)

    hud = fig.text(
        0.02, 0.96, "", va="top", ha="left",
        fontsize=10, family="monospace",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
    )

    ax_slider = plt.axes((0.12, 0.24, 0.52, 0.03))
    slider = Slider(ax_slider, "Time (s)", 0.0, max(t_end_s, 0.01), valinit=0.0)
    ax_play = plt.axes((0.12, 0.06, 0.08, 0.04))
    btn_play = Button(ax_play, "Play/Pause")

    filter_labels = [label for _, label in PHASOR_FILTERS]
    default_filter_idx = next(
        i for i, (fid, _) in enumerate(PHASOR_FILTERS) if fid == "moving_avg"
    )
    ax_filter = plt.axes((0.22, 0.02, 0.17, 0.20))
    radio_filter = RadioButtons(ax_filter, filter_labels, active=default_filter_idx)
    ax_filter.set_title("Phasor filter", fontsize=9)
    filter_label_to_id = {label: fid for fid, label in PHASOR_FILTERS}

    ax_smooth = plt.axes((0.42, 0.12, 0.22, 0.025))
    slider_smooth = Slider(ax_smooth, "Smooth", 0.0, 100.0, valinit=30.0)

    ax_invert = plt.axes((0.66, 0.14, 0.11, 0.07))
    check_invert = CheckButtons(ax_invert, ("Inv H", "Inv V"), (False, True))
    ax_invert.set_title("Invert", fontsize=9)

    ax_radio_h = plt.axes((0.70, 0.04, 0.10, 0.14))
    ax_radio_v = plt.axes((0.84, 0.04, 0.10, 0.14))
    radio_h = RadioButtons(ax_radio_h, ("x", "y", "z"), active=("x", "y", "z").index(h_axis))
    radio_v = RadioButtons(ax_radio_v, ("x", "y", "z"), active=("x", "y", "z").index(v_axis))
    ax_radio_h.set_title("H axis", fontsize=9)
    ax_radio_v.set_title("V axis", fontsize=9)

    state = {
        "playing": False,
        "playback_update": False,
        "axes_update": False,
        "phasor_mode": phasor_mode,
        "h_axis": h_axis,
        "v_axis": v_axis,
        "filter_mode": "moving_avg",
        "filter_strength": 30.0,
        "filter_cache_key": None,
        "filter_h": [],
        "filter_v": [],
        "filter_h_raw": [],
        "filter_v_raw": [],
        "linear_filter_cache_key": None,
        "linear_filter_h": [],
        "linear_filter_v": [],
        "linear_filter_h_raw": [],
        "linear_filter_v_raw": [],
        "invert_h": False,
        "invert_v": True,
    }

    def _plane_tag() -> str:
        return f"{state['h_axis']}{state['v_axis']}"

    def _refresh_accel_legend() -> None:
        h, v = state["h_axis"], state["v_axis"]
        tag = f"{h}{v}"
        primary = "Gravity" if state["phasor_mode"] == "gravity" else "Linear"
        legend = ax_accel.get_legend()
        if legend is not None:
            legend.remove()
        handles = [accel_arrow, accel_h_arrow, accel_v_arrow]
        labels = [f"{primary} {tag}", f"{primary} a_{h}", f"{primary} a_{v}"]
        if state["phasor_mode"] == "gravity":
            handles.append(linear_arrow)
            labels.append(f"Linear (no g) {tag}")
        ax_accel.legend(handles, labels, loc="upper right", fontsize=8)

    def apply_axes_ui() -> None:
        h, v = state["h_axis"], state["v_axis"]
        tag = f"{h}{v}"
        primary = "Gravity" if state["phasor_mode"] == "gravity" else "Linear accel"
        ax_accel.set_xlabel(f"{primary} a_{h} body (m/s²)")
        ax_accel.set_ylabel(f"{primary} a_{v} body (m/s²)")
        ax_accel.set_title(f"{primary} {tag.upper()} phasor (filtered)")
        _refresh_accel_legend()
        invalidate_filter_cache()

    def invalidate_filter_cache() -> None:
        state["filter_cache_key"] = None
        state["linear_filter_cache_key"] = None

    def rebuild_filter_cache() -> None:
        primary_vector = "gravity" if state["phasor_mode"] == "gravity" else "linear"
        key = (
            primary_vector,
            state["h_axis"],
            state["v_axis"],
            state["filter_mode"],
            round(state["filter_strength"], 1),
        )
        if state["filter_cache_key"] != key:
            h_raw, v_raw = _extract_phasor_series(
                frames, state["h_axis"], state["v_axis"], vector=primary_vector,
            )
            h_f, v_f = _smooth_phasor_series(
                h_raw,
                v_raw,
                mode=state["filter_mode"],
                strength=state["filter_strength"],
            )
            state["filter_h_raw"] = h_raw
            state["filter_v_raw"] = v_raw
            state["filter_h"] = h_f
            state["filter_v"] = v_f
            state["filter_cache_key"] = key

        linear_key = (
            "linear",
            state["h_axis"],
            state["v_axis"],
            state["filter_mode"],
            round(state["filter_strength"], 1),
        )
        if state["linear_filter_cache_key"] != linear_key:
            lh_raw, lv_raw = _extract_phasor_series(
                frames, state["h_axis"], state["v_axis"], vector="linear",
            )
            lh_f, lv_f = _smooth_phasor_series(
                lh_raw,
                lv_raw,
                mode=state["filter_mode"],
                strength=state["filter_strength"],
            )
            state["linear_filter_h_raw"] = lh_raw
            state["linear_filter_v_raw"] = lv_raw
            state["linear_filter_h"] = lh_f
            state["linear_filter_v"] = lv_f
            state["linear_filter_cache_key"] = linear_key

    def frame_index_at_time(t_s: float) -> int:
        target_us = t0_us + int(t_s * 1_000_000)
        lo, hi = 0, len(frames) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if frames[mid].ts_us <= target_us:
                lo = mid
            else:
                hi = mid - 1
        return lo

    def frame_at_time(t_s: float) -> Frame:
        return frames[frame_index_at_time(t_s)]

    def draw_frame(frame: Frame, t_s: float) -> None:
        bar_patch.set_xy(_bar_polygon_2d(frame.quat, "xy", BAR_HALF_M))

        h_axis = state["h_axis"]
        v_axis = state["v_axis"]
        plane_tag = _plane_tag()
        rebuild_filter_cache()
        idx = frame_index_at_time(t_s)

        has_primary = (
            0 <= idx < len(state["filter_h"])
            and state["filter_h"][idx] is not None
            and state["filter_v"][idx] is not None
        )
        has_linear = (
            state["phasor_mode"] == "gravity"
            and 0 <= idx < len(state["linear_filter_h"])
            and state["linear_filter_h"][idx] is not None
            and state["linear_filter_v"][idx] is not None
        )
        h_plot = v_plot = 0.0
        h_raw = v_raw = 0.0
        lin_h_plot = lin_v_plot = 0.0
        lin_h_raw = lin_v_raw = 0.0
        amag = amag_raw = 0.0
        lin_mag = lin_mag_raw = 0.0
        angle_deg = 0.0
        lin_angle_deg = 0.0
        if has_primary:
            h_val = state["filter_h"][idx]  # type: ignore[index]
            v_val = state["filter_v"][idx]  # type: ignore[index]
            if state["filter_h_raw"][idx] is not None:
                h_raw = state["filter_h_raw"][idx]  # type: ignore[index]
                v_raw = state["filter_v_raw"][idx]  # type: ignore[index]
            h_plot = -h_val if state["invert_h"] else h_val
            v_plot = -v_val if state["invert_v"] else v_val
            amag = math.hypot(h_plot, v_plot)
            amag_raw = math.hypot(h_raw, v_raw)
            angle_deg = math.degrees(math.atan2(v_plot, h_plot))

            accel_h_arrow.set_offsets([[0, 0]])
            accel_h_arrow.set_UVC([h_plot], [0.0])
            accel_v_arrow.set_offsets([[0, 0]])
            accel_v_arrow.set_UVC([0.0], [v_plot])
            accel_arrow.set_offsets([[0, 0]])
            accel_arrow.set_UVC([h_plot], [v_plot])
        else:
            for arrow in (accel_arrow, accel_h_arrow, accel_v_arrow):
                arrow.set_offsets([[0, 0]])
                arrow.set_UVC([0.0], [0.0])

        if has_linear:
            lh_val = state["linear_filter_h"][idx]  # type: ignore[index]
            lv_val = state["linear_filter_v"][idx]  # type: ignore[index]
            if state["linear_filter_h_raw"][idx] is not None:
                lin_h_raw = state["linear_filter_h_raw"][idx]  # type: ignore[index]
                lin_v_raw = state["linear_filter_v_raw"][idx]  # type: ignore[index]
            lin_h_plot = -lh_val if state["invert_h"] else lh_val
            lin_v_plot = -lv_val if state["invert_v"] else lv_val
            lin_mag = math.hypot(lin_h_plot, lin_v_plot)
            lin_mag_raw = math.hypot(lin_h_raw, lin_v_raw)
            lin_angle_deg = math.degrees(math.atan2(lin_v_plot, lin_h_plot))
            linear_arrow.set_offsets([[0, 0]])
            linear_arrow.set_UVC([lin_h_plot], [lin_v_plot])
        else:
            linear_arrow.set_offsets([[0, 0]])
            linear_arrow.set_UVC([0.0], [0.0])

        filter_label = next(
            lbl for fid, lbl in PHASOR_FILTERS if fid == state["filter_mode"]
        )
        omega = frame.omega_rad_s
        omega_s = f"{omega:.3f}" if omega is not None else "—"
        primary_name = "g" if state["phasor_mode"] == "gravity" else "a"
        hud_lines = [
            f"t = {t_s:.3f} s",
            f"filter = {filter_label} ({state['filter_strength']:.0f}%)",
            f"|{primary_name}_{plane_tag}| = {amag:.3f} m/s²"
            + (f"  (raw {amag_raw:.3f})" if state["filter_mode"] != "none" else ""),
            f"∠({primary_name}_{v_axis}, {primary_name}_{h_axis}) = {angle_deg:.1f}°",
        ]
        if has_linear:
            source = frame.linear_source
            hud_lines.append(
                f"|a_lin_{plane_tag}| = {lin_mag:.3f} m/s²"
                + (f"  (raw {lin_mag_raw:.3f})" if state["filter_mode"] != "none" else "")
                + f"  [{source}]"
            )
            hud_lines.append(f"∠(a_lin_{v_axis}, a_lin_{h_axis}) = {lin_angle_deg:.1f}°")
        hud_lines.append(f"|ω| (from quat) = {omega_s} rad/s")
        if has_primary:
            hud_lines.append(
                f"{primary_name}_{plane_tag} = ({h_axis}={h_plot:.3f}, {v_axis}={v_plot:.3f})"
            )
        if has_linear:
            hud_lines.append(
                f"a_lin_{plane_tag} = ({h_axis}={lin_h_plot:.3f}, {v_axis}={lin_v_plot:.3f})"
            )
        if not has_primary and not has_linear:
            hud_lines = [f"t = {t_s:.3f} s", "(no IMU vector sample)"]
        hud.set_text("\n".join(hud_lines))

    def on_slider(val: float) -> None:
        if not state["playback_update"]:
            state["playing"] = False
        draw_frame(frame_at_time(val), val)
        fig.canvas.draw_idle()

    def set_time(t_s: float, *, from_playback: bool) -> None:
        t_s = max(0.0, min(t_end_s, t_s))
        if from_playback:
            state["playback_update"] = True
        try:
            if abs(slider.val - t_s) > 1e-9:
                slider.set_val(t_s)
            else:
                draw_frame(frame_at_time(t_s), t_s)
                fig.canvas.draw_idle()
        finally:
            state["playback_update"] = False

    def _start_playback_clock() -> None:
        """Anchor wall clock to current slider position for real-time playback."""
        state["playback_wall_t0"] = time.perf_counter()
        state["playback_rec_t0"] = slider.val

    def set_playing(playing: bool) -> None:
        if playing and not state["playing"]:
            _start_playback_clock()
        state["playing"] = playing

    def _other_axis(keep: str, avoid: str) -> str:
        for axis in "xyz":
            if axis != avoid:
                return axis
        return keep

    def _set_radio(radio: RadioButtons, axis: str) -> None:
        radio.set_active("xyz".index(axis))

    def on_h_axis(label: str) -> None:
        if state["axes_update"]:
            return
        new_h = label
        if new_h == state["v_axis"]:
            state["axes_update"] = True
            try:
                new_v = _other_axis(state["v_axis"], new_h)
                state["v_axis"] = new_v
                _set_radio(radio_v, new_v)
            finally:
                state["axes_update"] = False
        state["h_axis"] = new_h
        apply_axes_ui()
        draw_frame(frame_at_time(slider.val), slider.val)
        fig.canvas.draw_idle()

    def on_v_axis(label: str) -> None:
        if state["axes_update"]:
            return
        new_v = label
        if new_v == state["h_axis"]:
            state["axes_update"] = True
            try:
                new_h = _other_axis(state["h_axis"], new_v)
                state["h_axis"] = new_h
                _set_radio(radio_h, new_h)
            finally:
                state["axes_update"] = False
        state["v_axis"] = new_v
        apply_axes_ui()
        draw_frame(frame_at_time(slider.val), slider.val)
        fig.canvas.draw_idle()

    def on_filter_mode(label: str) -> None:
        state["filter_mode"] = filter_label_to_id[label]
        invalidate_filter_cache()
        draw_frame(frame_at_time(slider.val), slider.val)
        fig.canvas.draw_idle()

    def on_smooth_strength(val: float) -> None:
        state["filter_strength"] = float(val)
        invalidate_filter_cache()
        draw_frame(frame_at_time(slider.val), slider.val)
        fig.canvas.draw_idle()

    def on_invert(_label: str) -> None:
        inv_h, inv_v = check_invert.get_status()
        state["invert_h"] = bool(inv_h)
        state["invert_v"] = bool(inv_v)
        draw_frame(frame_at_time(slider.val), slider.val)
        fig.canvas.draw_idle()

    def on_play(_event) -> None:
        set_playing(not state["playing"])

    slider.on_changed(on_slider)
    radio_h.on_clicked(on_h_axis)
    radio_v.on_clicked(on_v_axis)
    radio_filter.on_clicked(on_filter_mode)
    slider_smooth.on_changed(on_smooth_strength)
    check_invert.on_clicked(on_invert)

    def on_key(event) -> None:
        if event.key == " ":
            set_playing(not state["playing"])
        elif event.key == "left":
            state["playing"] = False
            slider.set_val(max(0.0, slider.val - 0.05))
        elif event.key == "right":
            state["playing"] = False
            slider.set_val(min(t_end_s, slider.val + 0.05))

    fig.canvas.mpl_connect("key_press_event", on_key)
    btn_play.on_clicked(on_play)

    interval_ms = 33

    def update(_i: int) -> tuple:
        if state["playing"]:
            elapsed_s = time.perf_counter() - state["playback_wall_t0"]
            next_t = state["playback_rec_t0"] + elapsed_s * speed
            if next_t >= t_end_s:
                next_t = 0.0
                state["playback_rec_t0"] = 0.0
                state["playback_wall_t0"] = time.perf_counter()
            set_time(next_t, from_playback=True)
        return (bar_patch, accel_arrow, accel_h_arrow, accel_v_arrow, linear_arrow, hud)

    apply_axes_ui()
    draw_frame(frames[0], 0.0)
    ani = animation.FuncAnimation(fig, update, interval=interval_ms, blit=False, cache_frame_data=False)
    _ = ani  # keep reference
    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="2D spin + linear accel viewer")
    parser.add_argument(
        "capture",
        nargs="?",
        default="motorSpinFinal.jsonl",
        help="JSONL capture (default: motorSpinFinal.jsonl)",
    )
    parser.add_argument(
        "--spin-axis",
        choices=("x", "y", "z"),
        default="z",
        help="Rotation axis to highlight on spin panel (default: z)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Wall-clock playback multiplier: 1.0 = 1 s recording per 1 s real time (default: 1.0)",
    )
    parser.add_argument(
        "--accel-axes",
        type=_parse_accel_axes,
        default=("y", "z"),
        metavar="HV",
        help="Phasor horizontal+vertical body axes, two distinct letters from xyz (default: yz)",
    )
    parser.add_argument(
        "--accel-limit",
        type=float,
        default=ACCEL_LIMIT_MPS2,
        help="Half-range of accel phasor axes in m/s² (default: 0.5)",
    )
    parser.add_argument(
        "--linear-limit",
        type=float,
        default=None,
        help="Half-range for linear (no-g) arrow overlay in m/s² (default: same as --accel-limit)",
    )
    parser.add_argument(
        "--no-kf",
        action="store_true",
        help="Disable Kalman smoothing on quat + accel streams",
    )
    parser.add_argument(
        "--kf-accel-q",
        type=float,
        default=0.03,
        help="Kalman process noise for accel (default: 0.03)",
    )
    parser.add_argument(
        "--kf-accel-r",
        type=float,
        default=0.2,
        help="Kalman measurement noise for accel (default: 0.2)",
    )
    parser.add_argument(
        "--kf-quat-q",
        type=float,
        default=0.002,
        help="Kalman process noise for quaternion (default: 0.002)",
    )
    parser.add_argument(
        "--kf-quat-r",
        type=float,
        default=0.08,
        help="Kalman measurement noise for quaternion (default: 0.08)",
    )
    args = parser.parse_args()

    path = Path(args.capture)
    if not path.is_file():
        print(f"File not found: {path}", file=sys.stderr)
        return 1

    kf = KfConfig(
        enabled=not args.no_kf,
        accel_q=args.kf_accel_q,
        accel_r=args.kf_accel_r,
        quat_q=args.kf_quat_q,
        quat_r=args.kf_quat_r,
    )

    frames, phasor_mode = _load_frames(path, kf=kf)
    title = path.name
    accel_limit = args.accel_limit
    if phasor_mode == "gravity" and accel_limit == ACCEL_LIMIT_MPS2:
        accel_limit = 12.0
    linear_limit = args.linear_limit if args.linear_limit is not None else accel_limit
    print(f"Loaded {len(frames)} quat frames from {path}")
    print(f"Phasor mode: {phasor_mode} (wire type 1)")
    if phasor_mode == "gravity":
        sources = {frame.linear_source for frame in frames if frame.linear_source != "none"}
        if sources:
            print(f"Linear (no g) source(s): {', '.join(sorted(sources))}")
        else:
            print("Linear (no g): no derived samples")
    if kf.enabled:
        print(
            f"Kalman smoothing: on "
            f"(accel q={kf.accel_q}, r={kf.accel_r}; quat q={kf.quat_q}, r={kf.quat_r})"
        )
    else:
        print("Kalman smoothing: off")
    print("Controls: Play/Pause (Space) | timeline | H/V axes | phasor filter + Smooth slider")
    print(f"Accel phasor axes: horizontal={args.accel_axes[0]}, vertical={args.accel_axes[1]}")
    run_viewer(
        frames,
        phasor_mode=phasor_mode,
        spin_axis=args.spin_axis,
        speed=args.speed,
        title=title,
        accel_limit=accel_limit,
        linear_limit=linear_limit,
        accel_axes=args.accel_axes,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
