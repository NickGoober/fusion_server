#!/usr/bin/env python3
"""
Generate synthetic lever-arm calibration sensor stream data.

Geometry (body frame: +X right, +Y forward, +Z up):
  - Device rotates about body +X through the rotation center (origin).
  - IMU is offset from the center (imu lever arm); its top (+Z) points toward center.
  - Optical flow sensor is at a similar radius, lens facing down (-Z).

Wire format: [sensor_type, timestamp, data_array] per line.

  0 — accel [x, y, z]  (or quat [x, y, z, w] from some firmware)
  1 — quat  [x, y, z, w]
  2 — flow  [dx, dy, quality]
  3 — radar [mm]

Example:
  py generate_cal_test_data.py -o cal_test.jsonl --duration 30
  py generate_cal_test_data.py -o hand_spin.jsonl --hand-spin --spin-axis z --duration 15
  py cal_lever_arm.py cal_test.jsonl --host 127.0.0.1 --omega 0
"""

from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

from typing import Callable

from sensor_stream import (
    SENSOR_ACCEL,
    SENSOR_FLOW,
    SENSOR_QUAT,
    SENSOR_RADAR,
    format_sample,
)

FLOW_RESOLUTION = 0.10
FLOW_NPIX = 35.0
FLOW_THETAPIX = 0.71674


def cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def quat_from_axis_angle(axis: tuple[float, float, float], angle_rad: float) -> dict[str, float]:
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12:
        return {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
    ax /= n
    ay /= n
    az /= n
    half = angle_rad * 0.5
    s = math.sin(half)
    return {"w": math.cos(half), "x": ax * s, "y": ay * s, "z": az * s}


def _smoothstep(u: float) -> float:
    u = max(0.0, min(1.0, u))
    return u * u * (3.0 - 2.0 * u)


def spin_axis_unit(axis: str) -> tuple[float, float, float]:
    if axis == "y":
        return (0.0, 1.0, 0.0)
    if axis == "z":
        return (0.0, 0.0, 1.0)
    return (1.0, 0.0, 0.0)


def omega_vec(axis: str, omega: float) -> tuple[float, float, float]:
    ux, uy, uz = spin_axis_unit(axis)
    return (ux * omega, uy * omega, uz * omega)


def omega_rad_s(t_s: float, *, base: float, variable: bool, amp: float, period_s: float) -> float:
    if not variable:
        return base
    return base + amp * math.sin(2.0 * math.pi * t_s / period_s)


def hand_spin_omega_rad_s(
    t_s: float,
    *,
    peak: float,
    duration_s: float,
    ramp_s: float = 2.0,
) -> float:
    """
    Hand-spun rate profile: ease-in, semi-constant plateau with wobble, ease-out.
    """
    if duration_s <= 2.0 * ramp_s:
        ramp_s = max(0.5, duration_s * 0.25)

    if t_s < ramp_s:
        return peak * _smoothstep(t_s / ramp_s)
    if t_s > duration_s - ramp_s:
        return peak * _smoothstep((duration_s - t_s) / ramp_s)

    wobble = (
        0.07 * peak * math.sin(2.0 * math.pi * t_s / 4.2)
        + 0.025 * peak * math.sin(2.0 * math.pi * t_s / 0.85)
    )
    return peak + wobble


def kinematic_accel(
    omega: tuple[float, float, float],
    omega_dot: tuple[float, float, float],
    arm: tuple[float, float, float],
) -> tuple[float, float, float]:
    """a = ω̇ × r + ω × (ω × r) at the IMU."""
    w_cross_r = cross(omega, arm)
    wdot_cross_r = cross(omega_dot, arm)
    centripetal = cross(omega, w_cross_r)
    return (
        wdot_cross_r[0] + centripetal[0],
        wdot_cross_r[1] + centripetal[1],
        wdot_cross_r[2] + centripetal[2],
    )


def _pack_lines_as_batches(lines: list[str], *, batch_s: float) -> list[str]:
    """Group single-sample wire lines into collar-style 1 s batch lines."""
    if batch_s <= 0.0:
        return lines

    rows: list[tuple[int, list]] = []
    for line in lines:
        sensor, ts, data = json.loads(line)
        rows.append((int(ts), [sensor, ts, data]))
    rows.sort(key=lambda item: item[0])

    batch_us = int(batch_s * 1_000_000)
    if not rows:
        return lines

    t0 = rows[0][0]
    batches: list[list] = []
    current: list = []
    bucket_end = t0 + batch_us
    for ts, row in rows:
        while ts >= bucket_end and current:
            batches.append(current)
            current = []
            bucket_end += batch_us
        current.append(row)
    if current:
        batches.append(current)

    return [json.dumps(batch, separators=(",", ":")) for batch in batches]


def flow_pixels_from_velocity(
    v_cam_bx: float,
    v_cam_by: float,
    gx: float,
    gy: float,
    range_m: float,
    dt_s: float,
) -> tuple[int, int]:
    z_g = max(range_m, 0.02)
    flow_scale = dt_s * FLOW_NPIX / FLOW_THETAPIX
    if flow_scale < 1e-6:
        return 0, 0
    meas_x = v_cam_bx / z_g - gy
    meas_y = v_cam_by / z_g + gx
    dpixelx = meas_x / FLOW_RESOLUTION
    dpixely = meas_y / FLOW_RESOLUTION
    return int(round(dpixelx)), int(round(dpixely))


def generate_stream(
    *,
    duration_s: float,
    imu_arm_m: tuple[float, float, float],
    flow_offset_m: tuple[float, float, float],
    range_mm: int,
    spin_axis: str,
    omega_profile: Callable[[float], float],
    quat_hz: float,
    accel_hz: float,
    flow_hz: float,
    radar_hz: float,
    noise_accel: float,
    noise_gyro: float,
    noise_flow: float,
    seed: int,
) -> list[str]:
    rng = random.Random(seed)
    lines: list[str] = []
    t0_us = 0
    theta = 0.0
    prev_quat_t_s = 0.0
    flow_dt_s = 1.0 / flow_hz
    flow_residue_x = 0.0
    flow_residue_y = 0.0
    axis = spin_axis_unit(spin_axis)

    r_imu = imu_arm_m
    r_flow = (
        r_imu[0] + flow_offset_m[0],
        r_imu[1] + flow_offset_m[1],
        r_imu[2] + flow_offset_m[2],
    )
    range_m = range_mm / 1000.0

    def emit(sensor: int, ts_us: int, payload: dict) -> None:
        lines.append(format_sample(sensor, ts_us, payload))

    def sample_times(hz: float) -> list[float]:
        if hz <= 0:
            return []
        dt = 1.0 / hz
        n = int(duration_s * hz)
        return [i * dt for i in range(n + 1)]

    def omega_at(t_s: float) -> float:
        return omega_profile(t_s)

    def omega_dot_at(t_s: float) -> float:
        dt = 0.002
        return (omega_at(t_s + dt) - omega_at(t_s - dt)) / (2.0 * dt)

    quat_times = sample_times(quat_hz)
    accel_times = sample_times(accel_hz)
    flow_times = sample_times(flow_hz)
    radar_times = sample_times(radar_hz)

    for t_s in quat_times:
        omega = omega_at(t_s)
        dt = t_s - prev_quat_t_s if t_s > prev_quat_t_s else (1.0 / quat_hz)
        theta += omega * dt
        prev_quat_t_s = t_s
        q = quat_from_axis_angle(axis, theta)
        emit(SENSOR_QUAT, t0_us + int(t_s * 1_000_000), q)

    for t_s in accel_times:
        omega = omega_at(t_s)
        omega_dot = omega_dot_at(t_s)
        w_vec = omega_vec(spin_axis, omega)
        wdot_vec = omega_vec(spin_axis, omega_dot)
        accel = kinematic_accel(w_vec, wdot_vec, r_imu)
        ax = accel[0] + rng.gauss(0.0, noise_accel)
        ay = accel[1] + rng.gauss(0.0, noise_accel)
        az = accel[2] + rng.gauss(0.0, noise_accel)
        emit(SENSOR_ACCEL, t0_us + int(t_s * 1_000_000), {"x": ax, "y": ay, "z": az})

    for t_s in flow_times:
        omega = omega_at(t_s)
        w_vec = omega_vec(spin_axis, omega)
        v_flow = cross(w_vec, r_flow)
        gx = w_vec[0] + rng.gauss(0.0, noise_gyro)
        gy = w_vec[1] + rng.gauss(0.0, noise_gyro)
        dx_f, dy_f = flow_pixels_from_velocity(
            v_flow[0], v_flow[1], gx, gy, range_m, flow_dt_s,
        )
        dx_f += int(round(rng.gauss(0.0, noise_flow)))
        dy_f += int(round(rng.gauss(0.0, noise_flow)))

        flow_residue_x += dx_f
        flow_residue_y += dy_f
        raw_dx = int(round(flow_residue_x))
        raw_dy = int(round(flow_residue_y))
        flow_residue_x -= raw_dx
        flow_residue_y -= raw_dy

        emit(
            SENSOR_FLOW,
            t0_us + int(t_s * 1_000_000),
            {"dx": raw_dx, "dy": raw_dy, "quality": 255},
        )

    for t_s in radar_times:
        emit(
            SENSOR_RADAR,
            t0_us + int(t_s * 1_000_000),
            {"mm": range_mm},
        )

    lines.sort(key=lambda line: json.loads(line)[1])
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate synthetic lever-arm calibration sensor stream",
    )
    parser.add_argument("-o", "--output", type=Path, default=Path("cal_test.jsonl"))
    parser.add_argument("--duration", type=float, default=30.0, help="Capture length [s]")
    parser.add_argument("--imu-arm-x", type=float, default=0.0, help="IMU X offset from CoR [m]")
    parser.add_argument("--imu-arm-y", type=float, default=0.020,
                        help="IMU Y offset from rotation center [m] (+Y forward)")
    parser.add_argument("--imu-arm-z", type=float, default=0.0, help="IMU Z offset from CoR [m]")
    parser.add_argument("--flow-offset-x", type=float, default=0.020,
                        help="Flow sensor X offset from IMU [m]")
    parser.add_argument("--flow-offset-z", type=float, default=-0.010,
                        help="Flow sensor Z offset from IMU [m] (negative = lens down)")
    parser.add_argument("--range-mm", type=int, default=550)
    parser.add_argument("--spin-axis", choices=("x", "y", "z"), default="x",
                        help="Body axis of rotation (z for barbell-style hand spin)")
    parser.add_argument("--hand-spin", action="store_true",
                        help="Semi-constant hand-spun rate (ramp, plateau wobble, ramp down)")
    parser.add_argument("--omega", type=float, default=0.06,
                        help="Spin rate [rad/s]; with --hand-spin this is the plateau peak")
    parser.add_argument("--variable-rate", action="store_true",
                        help="Modulate spin rate sinusoidally (ignored with --hand-spin)")
    parser.add_argument("--omega-amp", type=float, default=0.03,
                        help="Spin-rate amplitude when --variable-rate [rad/s]")
    parser.add_argument("--omega-period", type=float, default=8.0,
                        help="Spin-rate modulation period [s]")
    parser.add_argument("--batch-s", type=float, default=0.0,
                        help="Pack samples into N-second batch lines (e.g. 1.0 for collar captures)")
    parser.add_argument("--quat-hz", type=float, default=100.0)
    parser.add_argument("--accel-hz", type=float, default=80.0)
    parser.add_argument("--flow-hz", type=float, default=40.0)
    parser.add_argument("--radar-hz", type=float, default=20.0)
    parser.add_argument("--noise-accel", type=float, default=0.02, help="Accel noise σ [m/s²]")
    parser.add_argument("--noise-gyro", type=float, default=0.0,
                        help="Gyro noise σ [rad/s] injected into flow model only")
    parser.add_argument("--noise-flow", type=float, default=0.1, help="Flow pixel noise σ")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    imu_arm = (args.imu_arm_x, args.imu_arm_y, args.imu_arm_z)
    flow_offset = (args.flow_offset_x, 0.0, args.flow_offset_z)

    if args.hand_spin:
        duration_s = args.duration
        peak = args.omega if args.omega != 0.06 else 8.0

        def omega_profile(t_s: float) -> float:
            return hand_spin_omega_rad_s(
                t_s, peak=peak, duration_s=duration_s,
            )
    else:

        def omega_profile(t_s: float) -> float:
            return omega_rad_s(
                t_s,
                base=args.omega,
                variable=args.variable_rate,
                amp=args.omega_amp,
                period_s=args.omega_period,
            )

    lines = generate_stream(
        duration_s=args.duration,
        imu_arm_m=imu_arm,
        flow_offset_m=flow_offset,
        range_mm=args.range_mm,
        spin_axis=args.spin_axis,
        omega_profile=omega_profile,
        quat_hz=args.quat_hz,
        accel_hz=args.accel_hz,
        flow_hz=args.flow_hz,
        radar_hz=args.radar_hz,
        noise_accel=args.noise_accel,
        noise_gyro=args.noise_gyro,
        noise_flow=args.noise_flow,
        seed=args.seed,
    )

    if args.batch_s > 0.0:
        lines = _pack_lines_as_batches(lines, batch_s=args.batch_s)

    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {len(lines)} lines to {args.output}")
    print(f"Spin axis: {args.spin_axis}")
    print(f"Ground truth imu lever arm (center->IMU) m: {imu_arm}")
    print(f"Ground truth flow offset (IMU->flow): {flow_offset}")
    print(f"Ground truth flow lever arm (center->flow): "
          f"({imu_arm[0] + flow_offset[0]:.4f}, "
          f"{imu_arm[1] + flow_offset[1]:.4f}, "
          f"{imu_arm[2] + flow_offset[2]:.4f})")


if __name__ == "__main__":
    main()
