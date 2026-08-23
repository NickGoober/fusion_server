#!/usr/bin/env python3
"""
Optical flow replay viewer for collar captures.

Loads PMW3901 flow samples from JSONL recordings (wire batches or device JSON rows),
shows per-frame pixel deltas and integrated displacement for sensor and body axes,
with interactive measurement (span select on integrated plots).

Requires: pip install matplotlib numpy

Examples:
  python flow_viewer.py captures/freeMoveFB.jsonl
  python flow_viewer.py captures/freeMoveUD.jsonl --speed 2
  python flow_viewer.py capture.jsonl --height-m 0.65 --show-meters
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, CheckButtons, Slider, SpanSelector

from device_protocol import unpack_collar_wire_line
from flow_endian import normalize_flow_dx_dy
from lever_arm_config import FLOW_MOUNT_PITCH_X_RAD
from replay_capture import CaptureSeries
from sensor_stream import SENSOR_FLOW

# Crazyflie / fusion defaults (fusion.c fusion_config_defaults).
FLOW_SWAP_XY = True
FLOW_INVERT_X = True
FLOW_INVERT_Y = True
FLOW_RESOLUTION = 0.10
FLOW_FOV_DEG = 42.0
FLOW_NPIX = 35.0


@dataclass
class FlowPoint:
    t_s: float
    sensor_dx: int
    sensor_dy: int
    body_bx: int
    body_by: int
    body_bz: float
    quality: int


def _map_flow_body(dx: int, dy: int) -> tuple[int, int, float]:
    """Sensor pixels → collar body axes (matches fusion.c direct-flow mapping)."""
    raw_x, raw_y = int(dx), int(dy)
    bx = raw_y if FLOW_SWAP_XY else raw_x
    by = raw_x if FLOW_SWAP_XY else raw_y
    if FLOW_INVERT_X:
        bx = -bx
    if FLOW_INVERT_Y:
        by = -by
    bz = float(by) * math.cos(FLOW_MOUNT_PITCH_X_RAD)
    return bx, by, bz


def _wire_batches_from_line(line: str) -> list[list[Any]]:
    line = line.strip()
    if not line or line.startswith("{"):
        return []
    try:
        batch = json.loads(line)
    except json.JSONDecodeError:
        return []
    if not isinstance(batch, list):
        return []
    if batch and isinstance(batch[0], list):
        return [row for row in batch if isinstance(row, list)]
    if len(batch) >= 3 and len(batch) % 3 == 0:
        return [batch[i : i + 3] for i in range(0, len(batch), 3)]
    if len(batch) == 3:
        return [batch]
    return []


def _interp_range_mm(range_samples: list[tuple[float, int]], t_s: float) -> float | None:
    if not range_samples:
        return None
    if t_s <= range_samples[0][0]:
        return float(range_samples[0][1])
    if t_s >= range_samples[-1][0]:
        return float(range_samples[-1][1])
    for i in range(len(range_samples) - 1):
        t0, r0 = range_samples[i]
        t1, r1 = range_samples[i + 1]
        if t0 <= t_s <= t1:
            if t1 <= t0:
                return float(r0)
            alpha = (t_s - t0) / (t1 - t0)
            return r0 + alpha * (r1 - r0)
    return float(range_samples[-1][1])


def load_flow_points(
    path: Path,
    *,
    apply_endian_fix: bool = True,
    min_quality: int = 0,
) -> tuple[list[FlowPoint], list[tuple[float, int]]]:
    """Load flow time series and optional radar range (t_s, mm)."""
    series = CaptureSeries()
    wire_flow: list[tuple[int, dict[str, int]]] = []

    range_wire: list[tuple[int, int]] = []

    with path.open(encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith("{"):
                try:
                    row = json.loads(stripped)
                except json.JSONDecodeError:
                    continue
                if isinstance(row, dict) and not row.get("_fusion_record"):
                    from replay_capture import parse_device_row
                    parse_device_row(row, series)
                continue

            for sample in unpack_collar_wire_line(stripped):
                if sample.sensor == SENSOR_FLOW:
                    wire_flow.append((sample.ts_us, dict(sample.data)))
                elif sample.sensor == 3:
                    mm = sample.data.get("mm")
                    if mm is not None:
                        range_wire.append((sample.ts_us, int(mm)))

            for row in _wire_batches_from_line(stripped):
                if len(row) < 3:
                    continue
                sensor = int(row[0])
                ts_us = int(row[1])
                payload = row[2]
                if sensor == 3 and isinstance(payload, list) and payload:
                    range_wire.append((ts_us, int(payload[0])))
                if sensor != SENSOR_FLOW:
                    continue
                payload = row[2]
                if not isinstance(payload, list) or len(payload) < 2:
                    continue
                ts_us = int(row[1])
                quality = int(payload[2]) if len(payload) >= 3 else 255
                wire_flow.append((
                    ts_us,
                    {"dx": int(payload[0]), "dy": int(payload[1]), "quality": quality},
                ))

    points: list[FlowPoint] = []
    t0_us: int | None = None

    def append_sample(ts_us: int, dx: int, dy: int, quality: int) -> None:
        nonlocal t0_us
        if apply_endian_fix:
            dx, dy = normalize_flow_dx_dy(dx, dy)
        if quality < min_quality:
            return
        if t0_us is None:
            t0_us = ts_us
        t_s = (ts_us - t0_us) / 1e6
        bx, by, bz = _map_flow_body(dx, dy)
        points.append(FlowPoint(t_s, dx, dy, bx, by, bz, quality))

    for sample in series.flow:
        append_sample(
            int(sample.t_ms * 1000),
            int(sample.value["dx"]),
            int(sample.value["dy"]),
            int(sample.value.get("quality", 255)),
        )

    for ts_us, data in wire_flow:
        append_sample(
            ts_us,
            int(data["dx"]),
            int(data["dy"]),
            int(data.get("quality", 255)),
        )

    points.sort(key=lambda p: p.t_s)

    # Dedupe identical timestamps (keep last).
    deduped: list[FlowPoint] = []
    for pt in points:
        if deduped and pt.t_s == deduped[-1].t_s:
            deduped[-1] = pt
        else:
            deduped.append(pt)
    points = deduped

    range_series = [(s.t_ms / 1000.0, int(s.value)) for s in series.range_mm]
    if wire_flow and t0_us is None:
        t0_us = min(ts for ts, _ in wire_flow)
    for ts_us, mm in range_wire:
        if t0_us is None:
            t0_us = ts_us
        range_series.append(((ts_us - t0_us) / 1e6, mm))
    range_series.sort(key=lambda pair: pair[0])
    return points, range_series


def _meters_per_pixel(height_m: float) -> float:
    fov_rad = math.radians(FLOW_FOV_DEG / FLOW_NPIX)
    return height_m * math.tan(fov_rad) * FLOW_RESOLUTION


def _mpp_array(heights_m: np.ndarray) -> np.ndarray:
    fov_rad = math.radians(FLOW_FOV_DEG / FLOW_NPIX)
    return heights_m * math.tan(fov_rad) * FLOW_RESOLUTION


def _heights_at_times(
    t_s: np.ndarray,
    range_series: list[tuple[float, int]],
    *,
    fallback_m: float,
) -> np.ndarray:
    if len(t_s) == 0:
        return np.array([], dtype=float)
    if not range_series:
        return np.full(len(t_s), fallback_m, dtype=float)
    out = np.empty(len(t_s), dtype=float)
    for i, ts in enumerate(t_s):
        mm = _interp_range_mm(range_series, float(ts))
        out[i] = (mm / 1000.0) if mm is not None else fallback_m
    return out


def _flow_to_meters(deltas: np.ndarray, heights_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    delta_m = deltas.astype(float) * _mpp_array(heights_m)
    return delta_m, np.cumsum(delta_m)


def _span_displacement_m(
    deltas: np.ndarray,
    heights_m: np.ndarray,
    i0: int,
    i1: int,
) -> float:
    mpp = _mpp_array(heights_m[i0 : i1 + 1])
    return float(np.sum(deltas[i0 : i1 + 1].astype(float) * mpp))


def _axis_stats(
    deltas: np.ndarray,
    integrated: np.ndarray,
    *,
    unit: str,
) -> dict[str, float]:
    if len(deltas) == 0:
        return {
            "net": 0.0,
            "path": 0.0,
            "peak_pos": 0.0,
            "peak_neg": 0.0,
            "range": 0.0,
            "unit": unit,
        }
    net = float(integrated[-1])
    path = float(np.sum(np.abs(deltas)))
    peak_pos = float(np.max(integrated))
    peak_neg = float(np.min(integrated))
    return {
        "net": net,
        "path": path,
        "peak_pos": peak_pos,
        "peak_neg": peak_neg,
        "range": peak_pos - peak_neg,
        "unit": unit,
    }


def _format_stats(label: str, stats: dict[str, float]) -> str:
    u = stats["unit"]
    return (
        f"{label}: net={stats['net']:+.2f} {u}  path={stats['path']:.2f} {u}  "
        f"range=[{stats['peak_neg']:.2f}, {stats['peak_pos']:.2f}] {u}"
    )


def run_viewer(
    points: list[FlowPoint],
    range_series: list[tuple[float, int]],
    *,
    height_m: float | None,
    show_meters: bool,
    speed: float,
) -> None:
    if not points:
        print("No flow samples found in capture.")
        return

    t = np.array([p.t_s for p in points], dtype=float)
    sensor_dx = np.array([p.sensor_dx for p in points], dtype=int)
    sensor_dy = np.array([p.sensor_dy for p in points], dtype=int)
    body_bx = np.array([p.body_bx for p in points], dtype=int)
    body_bz = np.array([p.body_bz for p in points], dtype=float)

    cum_sensor_dx = np.cumsum(sensor_dx)
    cum_sensor_dy = np.cumsum(sensor_dy)
    cum_body_bx = np.cumsum(body_bx)
    cum_body_bz = np.cumsum(body_bz)

    default_height = height_m
    if default_height is None and range_series:
        default_height = _interp_range_mm(range_series, t[len(t) // 2]) / 1000.0
    if default_height is None:
        default_height = 0.60

    fixed_height_m = height_m
    heights_m = (
        np.full(len(t), fixed_height_m, dtype=float)
        if fixed_height_m is not None
        else _heights_at_times(t, range_series, fallback_m=default_height)
    )
    _, cum_sensor_dx_m = _flow_to_meters(sensor_dx, heights_m)
    _, cum_body_bz_m = _flow_to_meters(body_bz, heights_m)

    axes_config = [
        ("sensor_dx", "Sensor Δx", sensor_dx, cum_sensor_dx, "px"),
        ("sensor_dy", "Sensor Δy", sensor_dy, cum_sensor_dy, "px"),
        ("body_bx", "Body Δx (+right)", body_bx, cum_body_bx, "px"),
        ("body_bz", "Body Δz (+forward)", body_bz, cum_body_bz, "px"),
    ]

    fig, axes = plt.subplots(2, 4, figsize=(14, 7), sharex="col")
    fig.subplots_adjust(left=0.06, right=0.98, top=0.96, bottom=0.40, hspace=0.35, wspace=0.28)

    delta_lines: list[Any] = []
    cum_lines: list[Any] = []
    cum_lines_m: list[Any] = []
    vlines_delta: list[Any] = []
    vlines_cum: list[Any] = []
    span_artists: list[list[Any]] = []

    stats_by_key: dict[str, dict[str, float]] = {}
    state = {
        "playing": False,
        "show_meters": show_meters,
        "fixed_height_m": fixed_height_m,
        "heights_m": heights_m,
        "default_height": default_height,
    }

    def heights_for_display() -> np.ndarray:
        if state["fixed_height_m"] is not None:
            return np.full(len(t), state["fixed_height_m"], dtype=float)
        return state["heights_m"]

    def apply_unit_display() -> None:
        show_m = state["show_meters"]
        heights = heights_for_display()
        unit = "m" if show_m else "px"
        for col, (key, title, deltas, cum, raw_unit) in enumerate(axes_config):
            dp = deltas.astype(float)
            cp = cum.astype(float)
            if show_m and raw_unit == "px":
                dp, cp = _flow_to_meters(deltas, heights)
                axes[0, col].set_ylabel("m / frame")
                axes[1, col].set_ylabel("m")
            else:
                axes[0, col].set_ylabel("px / frame")
                axes[1, col].set_ylabel("px (— m est.)")

            delta_lines[col].set_ydata(dp)
            cum_lines[col].set_ydata(cp)
            if cum_lines_m[col] is not None:
                cum_lines_m[col].set_visible(not show_m)

            stats = _axis_stats(
                dp,
                cp if show_m else cum.astype(float),
                unit=unit if show_m else "px",
            )
            stats_by_key[key] = stats
            axes[1, col].set_title(
                f"{title} — integrated\n"
                f"net {stats['net']:+.2f} {stats['unit']}  path {stats['path']:.1f}  "
                f"range {stats['range']:.2f}",
                fontsize=9,
            )
            for ax in (axes[0, col], axes[1, col]):
                ax.relim()
                ax.autoscale_view(scalex=False)

    for col, (key, title, deltas, cum, raw_unit) in enumerate(axes_config):
        ax_d = axes[0, col]
        ax_c = axes[1, col]

        line_d, = ax_d.plot(t, deltas.astype(float), color="#2563eb", linewidth=0.9, label="Δ/frame")
        v_d = ax_d.axvline(0, color="#ef4444", linewidth=1.0, alpha=0.7)
        ax_d.axhline(0, color="#94a3b8", linewidth=0.6)
        ax_d.set_title(f"{title} — delta")
        ax_d.grid(True, alpha=0.3)

        line_c, = ax_c.plot(t, cum.astype(float), color="#16a34a", linewidth=1.2, label="∫Δ")
        line_c_m = None
        if raw_unit == "px" and not show_meters:
            _, cum_m_overlay = _flow_to_meters(deltas, heights_m)
            overlay_label = (
                f"∫Δ radar-scaled"
                if fixed_height_m is None and range_series
                else f"∫Δ @ {default_height:.2f}m"
            )
            line_c_m, = ax_c.plot(
                t, cum_m_overlay, color="#f59e0b", linewidth=1.0,
                linestyle="--", alpha=0.85, label=overlay_label,
            )
        ax_c.axhline(0, color="#94a3b8", linewidth=0.6)
        v_c = ax_c.axvline(0, color="#ef4444", linewidth=1.0, alpha=0.7)
        ax_c.set_title(f"{title} — integrated")
        ax_c.grid(True, alpha=0.3)
        if line_c_m is not None:
            ax_c.legend(loc="upper left", fontsize=7)

        delta_lines.append(line_d)
        cum_lines.append(line_c)
        cum_lines_m.append(line_c_m)
        vlines_delta.append(v_d)
        vlines_cum.append(v_c)
        span_artists.append([])

        def make_on_select(col_idx: int):
            def on_select(xmin: float, xmax: float) -> None:
                if xmax < xmin:
                    xmin, xmax = xmax, xmin
                mask = (t >= xmin) & (t <= xmax)
                if not np.any(mask):
                    return
                i0 = int(np.argmax(mask))
                i1 = int(len(mask) - 1 - np.argmax(mask[::-1]))
                _, _, deltas_arr, cum_arr, _ = axes_config[col_idx]
                if state["show_meters"]:
                    delta_span = _span_displacement_m(
                        deltas_arr, heights_for_display(), i0, i1,
                    )
                    unit = "m"
                else:
                    delta_span = float(cum_arr[i1] - cum_arr[i0])
                    unit = "px"
                for art in span_artists[col_idx]:
                    art.remove()
                span_artists[col_idx].clear()
                span_artists[col_idx].append(
                    ax_c.axvspan(xmin, xmax, color="#fbbf24", alpha=0.25),
                )
                span_artists[col_idx].append(
                    ax_c.text(
                        0.02, 0.95,
                        f"Δ[{xmin:.2f},{xmax:.2f}s] = {delta_span:+.3f} {unit}",
                        transform=ax_c.transAxes,
                        fontsize=8,
                        va="top",
                        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
                    ),
                )
                fig.canvas.draw_idle()
            return on_select

        SpanSelector(
            ax_c,
            make_on_select(col),
            "horizontal",
            useblit=True,
            props=dict(alpha=0.15, facecolor="#fbbf24"),
            button=0,
            minspan=0.02,
            interactive=True,
        )

    apply_unit_display()

    for ax in axes[1, :]:
        ax.set_xlabel("Time (s)")

    hud = fig.text(
        0.02, 0.14, "", va="bottom", ha="left", fontsize=8, family="monospace",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.9),
    )
    summary = fig.text(
        0.50, 0.14, "", va="bottom", ha="center", fontsize=8, family="monospace",
        bbox=dict(boxstyle="round", facecolor="#f1f5f9", alpha=0.9),
    )

    t_end = float(t[-1])
    ax_slider = plt.axes((0.12, 0.10, 0.55, 0.03))
    slider = Slider(ax_slider, "Time (s)", 0.0, max(t_end, 0.01), valinit=0.0)
    ax_play = plt.axes((0.12, 0.04, 0.10, 0.04))
    btn_play = Button(ax_play, "Play/Pause")
    ax_meters = plt.axes((0.70, 0.04, 0.12, 0.06))
    check_meters = CheckButtons(ax_meters, ["Show meters"], [show_meters])
    ax_meters.set_title("Units", fontsize=9)

    def _height_summary() -> str:
        if state["fixed_height_m"] is not None:
            return f"height={state['fixed_height_m']:.3f}m (fixed)"
        h = state["heights_m"]
        if len(h) == 0:
            return f"height≈{state['default_height']:.3f}m (default)"
        return f"height radar {h.min():.3f}–{h.max():.3f}m"

    def refresh_summary() -> None:
        lines = [
            f"Samples: {len(points)}  duration: {t_end:.2f}s  {_height_summary()}",
            _format_stats("sensor_dx", stats_by_key["sensor_dx"]),
            _format_stats("body_bx", stats_by_key["body_bx"]),
            _format_stats("body_bz", stats_by_key["body_bz"]),
        ]
        summary.set_text("\n".join(lines))

    def update_cursor(t_sel: float) -> None:
        idx = int(np.searchsorted(t, t_sel, side="right") - 1)
        idx = max(0, min(idx, len(t) - 1))
        for v in vlines_delta:
            v.set_xdata([t_sel, t_sel])
        for v in vlines_cum:
            v.set_xdata([t_sel, t_sel])
        p = points[idx]
        height_now = float(heights_for_display()[idx])
        if state["show_meters"]:
            int_sdx = float(cum_sensor_dx_m[idx])
            int_bz = float(cum_body_bz_m[idx])
            int_unit = "m"
        else:
            int_sdx = cum_sensor_dx[idx]
            int_bz = cum_body_bz[idx]
            int_unit = "px"
        hud.set_text(
            f"t={t_sel:.3f}s  q={p.quality}  height={height_now:.3f}m\n"
            f"sensor dx,dy={p.sensor_dx},{p.sensor_dy} px\n"
            f"body bx,bz={p.body_bx},{p.body_bz:.2f} px\n"
            f"∫ sensor_dx={int_sdx:+.4f}  ∫ body_bz={int_bz:+.4f} {int_unit}"
        )
        fig.canvas.draw_idle()

    def on_slider(val: float) -> None:
        update_cursor(float(val))

    def toggle_play(_event: Any) -> None:
        state["playing"] = not state["playing"]

    def on_meters(_label: str) -> None:
        state["show_meters"] = check_meters.get_status()[0]
        apply_unit_display()
        refresh_summary()
        update_cursor(float(slider.val))

    slider.on_changed(on_slider)
    btn_play.on_clicked(toggle_play)
    check_meters.on_clicked(on_meters)

    refresh_summary()
    update_cursor(0.0)

    print("Controls: timeline slider | Play/Pause | drag on integrated plots to measure Δ displacement")
    print(_format_stats("body_bx (LR proxy)", stats_by_key["body_bx"]))
    print(_format_stats("body_bz (FB proxy)", stats_by_key["body_bz"]))

    last_wall = time.monotonic()
    while plt.fignum_exists(fig.number):
        if state["playing"]:
            now = time.monotonic()
            dt = now - last_wall
            last_wall = now
            new_t = min(slider.val + dt * speed, t_end)
            slider.set_val(new_t)
        plt.pause(0.03)


def main() -> int:
    parser = argparse.ArgumentParser(description="Optical flow delta + integrated displacement viewer")
    parser.add_argument(
        "capture",
        nargs="?",
        default="captures/freeMoveFB.jsonl",
        help="JSONL capture with flow samples",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (default: 1.0)",
    )
    parser.add_argument(
        "--height-m",
        type=float,
        default=None,
        help="Override radar height for m conversion (default: per-sample radar)",
    )
    parser.add_argument(
        "--show-meters",
        action="store_true",
        help="Plot integrated/delta in meters using height and PMW3901 FOV model",
    )
    parser.add_argument(
        "--no-endian-fix",
        action="store_true",
        help="Do not apply runtime PMW3901 byte-swap normalization",
    )
    parser.add_argument(
        "--min-quality",
        type=int,
        default=0,
        help="Drop flow frames below this SQUAL (default: 0)",
    )
    args = parser.parse_args()

    path = Path(args.capture)
    if not path.is_file():
        print(f"Capture not found: {path}", file=sys.stderr)
        return 1

    points, range_series = load_flow_points(
        path,
        apply_endian_fix=not args.no_endian_fix,
        min_quality=args.min_quality,
    )
    if not points:
        print(f"No flow samples in {path}", file=sys.stderr)
        return 1

    print(f"Loaded {len(points)} flow samples from {path}")
    run_viewer(
        points,
        range_series,
        height_m=args.height_m,
        show_meters=args.show_meters,
        speed=max(args.speed, 0.01),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
