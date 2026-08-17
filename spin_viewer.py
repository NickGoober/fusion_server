#!/usr/bin/env python3
"""
2D replay viewer for collar spin captures.

Left panel: bar rotating from game-rotation quaternion (top-down XY).
Right panel: 2-axis linear-accel phasor (select axes with --accel-axes, e.g. zy).

Requires: pip install matplotlib numpy

Examples:
  python spin_viewer.py motorSpinFinal.jsonl
  python spin_viewer.py capture1.jsonl --accel-axes xy --speed 2
  python spin_viewer.py hand_spin_sim.jsonl --no-kf
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import animation
from matplotlib.widgets import Button, CheckButtons, RadioButtons, Slider

from device_protocol import unpack_collar_wire_line
from sensor_stream import SENSOR_ACCEL, SENSOR_QUAT, gyro_from_quat_pair

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
    accel: dict[str, float] | None
    omega_rad_s: float | None = None


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
) -> tuple[list[float | None], list[float | None]]:
    h_raw: list[float | None] = []
    v_raw: list[float | None] = []
    for frame in frames:
        if frame.accel is None:
            h_raw.append(None)
            v_raw.append(None)
        else:
            h_raw.append(frame.accel[h_axis])
            v_raw.append(frame.accel[v_axis])
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


def _load_frames(path: Path, *, kf: KfConfig | None = None) -> list[Frame]:
    quats: list[tuple[int, dict[str, float]]] = []
    accels: list[tuple[int, dict[str, float]]] = []

    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("{"):
                continue
            for sample in unpack_collar_wire_line(line):
                if sample.sensor == SENSOR_QUAT:
                    quats.append((sample.ts_us, sample.data))
                elif sample.sensor == SENSOR_ACCEL:
                    accels.append((sample.ts_us, sample.data))

    if not quats:
        raise SystemExit(f"No quaternions in {path}")

    quats.sort(key=lambda item: item[0])
    accels.sort(key=lambda item: item[0])

    if kf and kf.enabled:
        quats = _filter_quat_stream(quats, kf.quat_q, kf.quat_r)
        if accels:
            accels = _filter_accel_stream(accels, kf.accel_q, kf.accel_r)

    # Timeline on quat samples; attach nearest accel (if any).
    accel_idx = 0
    frames: list[Frame] = []
    for i, (ts, quat) in enumerate(quats):
        while accel_idx + 1 < len(accels) and accels[accel_idx + 1][0] <= ts:
            accel_idx += 1
        accel = None
        if accels:
            if accel_idx < len(accels):
                a_ts, a_data = accels[accel_idx]
                if i + 1 < len(quats):
                    next_ts = quats[i + 1][0]
                    if a_ts <= next_ts or abs(a_ts - ts) < abs(a_ts - next_ts):
                        accel = a_data
                elif abs(a_ts - ts) < 50_000:
                    accel = a_data

        omega = None
        if i > 0:
            prev_ts, prev_q = quats[i - 1]
            dt = (ts - prev_ts) / 1_000_000.0
            if 0.0 < dt < 0.1:
                g = gyro_from_quat_pair(prev_q, quat, dt)
                omega = math.sqrt(g["x"] ** 2 + g["y"] ** 2 + g["z"] ** 2)

        frames.append(Frame(ts_us=ts, quat=quat, accel=accel, omega_rad_s=omega))

    return frames


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
    spin_axis: str,
    speed: float,
    title: str,
    accel_limit: float,
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
    ax_accel.set_xlabel(f"a_{h_axis} body (m/s²)")
    ax_accel.set_ylabel(f"a_{v_axis} body (m/s²)")
    ax_accel.set_title(f"Linear accel {plane_tag.upper()} phasor (body frame)")

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
        label=f"a_{plane_tag} resultant",
    )
    accel_h_arrow = ax_accel.quiver(
        0, 0, 0, 0, **quiver_kw,
        color="#ef4444", width=0.009, headwidth=4, headlength=5,
        label=f"a_{h_axis}",
    )
    accel_v_arrow = ax_accel.quiver(
        0, 0, 0, 0, **quiver_kw,
        color="#3b82f6", width=0.009, headwidth=4, headlength=5,
        label=f"a_{v_axis}",
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
        i for i, (fid, _) in enumerate(PHASOR_FILTERS) if fid == "polar_ema"
    )
    ax_filter = plt.axes((0.22, 0.02, 0.17, 0.20))
    radio_filter = RadioButtons(ax_filter, filter_labels, active=default_filter_idx)
    ax_filter.set_title("Phasor filter", fontsize=9)
    filter_label_to_id = {label: fid for fid, label in PHASOR_FILTERS}

    ax_smooth = plt.axes((0.42, 0.12, 0.22, 0.025))
    slider_smooth = Slider(ax_smooth, "Smooth", 0.0, 100.0, valinit=65.0)

    ax_invert = plt.axes((0.66, 0.14, 0.11, 0.07))
    check_invert = CheckButtons(ax_invert, ("Inv H", "Inv V"), (False, False))
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
        "h_axis": h_axis,
        "v_axis": v_axis,
        "filter_mode": "polar_ema",
        "filter_strength": 65.0,
        "filter_cache_key": None,
        "filter_h": [],
        "filter_v": [],
        "filter_h_raw": [],
        "filter_v_raw": [],
        "invert_h": False,
        "invert_v": False,
    }

    def _plane_tag() -> str:
        return f"{state['h_axis']}{state['v_axis']}"

    def _refresh_accel_legend() -> None:
        h, v = state["h_axis"], state["v_axis"]
        tag = f"{h}{v}"
        legend = ax_accel.get_legend()
        if legend is not None:
            legend.remove()
        ax_accel.legend(
            [accel_arrow, accel_h_arrow, accel_v_arrow],
            [f"a_{tag} resultant", f"a_{h}", f"a_{v}"],
            loc="upper right",
            fontsize=8,
        )

    def apply_axes_ui() -> None:
        h, v = state["h_axis"], state["v_axis"]
        tag = f"{h}{v}"
        ax_accel.set_xlabel(f"a_{h} body (m/s²)")
        ax_accel.set_ylabel(f"a_{v} body (m/s²)")
        ax_accel.set_title(f"Linear accel {tag.upper()} phasor (filtered)")
        _refresh_accel_legend()
        invalidate_filter_cache()

    def invalidate_filter_cache() -> None:
        state["filter_cache_key"] = None

    def rebuild_filter_cache() -> None:
        key = (
            state["h_axis"],
            state["v_axis"],
            state["filter_mode"],
            round(state["filter_strength"], 1),
        )
        if state["filter_cache_key"] == key:
            return
        h_raw, v_raw = _extract_phasor_series(frames, state["h_axis"], state["v_axis"])
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

        h_val = v_val = 0.0
        h_raw = v_raw = 0.0
        amag = amag_raw = 0.0
        angle_deg = 0.0
        has_accel = (
            0 <= idx < len(state["filter_h"])
            and state["filter_h"][idx] is not None
            and state["filter_v"][idx] is not None
        )
        if has_accel:
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

        filter_label = next(
            lbl for fid, lbl in PHASOR_FILTERS if fid == state["filter_mode"]
        )
        omega = frame.omega_rad_s
        omega_s = f"{omega:.3f}" if omega is not None else "—"
        hud.set_text(
            f"t = {t_s:.3f} s\n"
            f"filter = {filter_label} ({state['filter_strength']:.0f}%)\n"
            f"|a_{plane_tag}| = {amag:.3f} m/s²"
            + (f"  (raw {amag_raw:.3f})" if state["filter_mode"] != "none" else "")
            + f"\n∠(a_{v_axis}, a_{h_axis}) = {angle_deg:.1f}°\n"
            f"|ω| (from quat) = {omega_s} rad/s\n"
            f"a_{plane_tag} = ({h_axis}={h_plot:.3f}, {v_axis}={v_plot:.3f})"
            if has_accel
            else f"t = {t_s:.3f} s\n(no accel sample)"
        )

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
        return (bar_patch, accel_arrow, accel_h_arrow, accel_v_arrow, hud)

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
        default=("z", "y"),
        metavar="HV",
        help="Phasor horizontal+vertical body axes, two distinct letters from xyz (default: zy)",
    )
    parser.add_argument(
        "--accel-limit",
        type=float,
        default=ACCEL_LIMIT_MPS2,
        help="Half-range of accel phasor axes in m/s² (default: 0.5)",
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

    frames = _load_frames(path, kf=kf)
    title = path.name
    print(f"Loaded {len(frames)} quat frames from {path}")
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
        spin_axis=args.spin_axis,
        speed=args.speed,
        title=title,
        accel_limit=args.accel_limit,
        accel_axes=args.accel_axes,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
