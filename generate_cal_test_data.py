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
  py cal_lever_arm.py cal_test.jsonl --host 127.0.0.1 --omega 0
"""

from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

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


def omega_rad_s(t_s: float, *, base: float, variable: bool, amp: float, period_s: float) -> float:
    if not variable:
        return base
    return base + amp * math.sin(2.0 * math.pi * t_s / period_s)


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
    omega_base: float,
    variable_rate: bool,
    omega_amp: float,
    omega_period_s: float,
    quat_hz: float,
    accel_hz: float,
    flow_hz: float,
    radar_hz: float,
    noise_accel: float,
    noise_flow: float,
    seed: int,
) -> list[str]:
    rng = random.Random(seed)
    lines: list[str] = []
    t0_us = 0
    theta = 0.0
    prev_t_s = 0.0
    flow_dt_s = 1.0 / flow_hz
    flow_residue_x = 0.0
    flow_residue_y = 0.0

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

    quat_times = sample_times(quat_hz)
    accel_times = sample_times(accel_hz)
    flow_times = sample_times(flow_hz)
    radar_times = sample_times(radar_hz)

    for t_s in quat_times:
        omega = omega_rad_s(
            t_s,
            base=omega_base,
            variable=variable_rate,
            amp=omega_amp,
            period_s=omega_period_s,
        )
        dt = t_s - prev_t_s if t_s > prev_t_s else (1.0 / quat_hz)
        theta += omega * dt
        prev_t_s = t_s
        q = quat_from_axis_angle((1.0, 0.0, 0.0), theta)
        emit(SENSOR_QUAT, t0_us + int(t_s * 1_000_000), q)

    prev_t_s = 0.0
    for t_s in accel_times:
        omega = omega_rad_s(
            t_s,
            base=omega_base,
            variable=variable_rate,
            amp=omega_amp,
            period_s=omega_period_s,
        )
        w_vec = (omega, 0.0, 0.0)
        # Centripetal acceleration at IMU: ω × (ω × r)
        w_cross_r = cross(w_vec, r_imu)
        accel = cross(w_vec, w_cross_r)
        ax = accel[0] + rng.gauss(0.0, noise_accel)
        ay = accel[1] + rng.gauss(0.0, noise_accel)
        az = accel[2] + rng.gauss(0.0, noise_accel)
        emit(SENSOR_ACCEL, t0_us + int(t_s * 1_000_000), {"x": ax, "y": ay, "z": az})

    for i, t_s in enumerate(flow_times):
        omega = omega_rad_s(
            t_s,
            base=omega_base,
            variable=variable_rate,
            amp=omega_amp,
            period_s=omega_period_s,
        )
        w_vec = (omega, 0.0, 0.0)
        v_flow = cross(w_vec, r_flow)
        # mm_flow.c body velocity model (inverse used for pixel synthesis)
        dx_f, dy_f = flow_pixels_from_velocity(
            v_flow[0], v_flow[1], omega, 0.0, range_m, flow_dt_s,
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
    parser.add_argument("--imu-arm-y", type=float, default=0.020,
                        help="IMU Y offset from rotation center [m] (+Y forward)")
    parser.add_argument("--flow-offset-x", type=float, default=0.020,
                        help="Flow sensor X offset from IMU [m]")
    parser.add_argument("--flow-offset-z", type=float, default=-0.010,
                        help="Flow sensor Z offset from IMU [m] (negative = lens down)")
    parser.add_argument("--range-mm", type=int, default=550)
    parser.add_argument("--omega", type=float, default=0.06,
                        help="Base spin rate about +X [rad/s]")
    parser.add_argument("--variable-rate", action="store_true",
                        help="Modulate spin rate sinusoidally")
    parser.add_argument("--omega-amp", type=float, default=0.03,
                        help="Spin-rate amplitude when --variable-rate [rad/s]")
    parser.add_argument("--omega-period", type=float, default=8.0,
                        help="Spin-rate modulation period [s]")
    parser.add_argument("--quat-hz", type=float, default=100.0)
    parser.add_argument("--accel-hz", type=float, default=80.0)
    parser.add_argument("--flow-hz", type=float, default=40.0)
    parser.add_argument("--radar-hz", type=float, default=20.0)
    parser.add_argument("--noise-accel", type=float, default=0.02, help="Accel noise σ [m/s²]")
    parser.add_argument("--noise-flow", type=float, default=0.1, help="Flow pixel noise σ")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    imu_arm = (0.0, args.imu_arm_y, 0.0)
    flow_offset = (args.flow_offset_x, 0.0, args.flow_offset_z)

    lines = generate_stream(
        duration_s=args.duration,
        imu_arm_m=imu_arm,
        flow_offset_m=flow_offset,
        range_mm=args.range_mm,
        omega_base=args.omega,
        variable_rate=args.variable_rate,
        omega_amp=args.omega_amp,
        omega_period_s=args.omega_period,
        quat_hz=args.quat_hz,
        accel_hz=args.accel_hz,
        flow_hz=args.flow_hz,
        radar_hz=args.radar_hz,
        noise_accel=args.noise_accel,
        noise_flow=args.noise_flow,
        seed=args.seed,
    )

    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {len(lines)} samples to {args.output}")
    print(f"Ground truth imu lever arm (center->IMU): {imu_arm}")
    print(f"Ground truth flow offset (IMU->flow): {flow_offset}")
    print(f"Ground truth flow lever arm (center->flow): "
          f"({imu_arm[0] + flow_offset[0]:.4f}, "
          f"{imu_arm[1] + flow_offset[1]:.4f}, "
          f"{imu_arm[2] + flow_offset[2]:.4f})")


if __name__ == "__main__":
    main()
