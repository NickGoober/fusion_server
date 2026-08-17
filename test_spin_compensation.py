#!/usr/bin/env python3
"""
Test IMU lever-arm compensation on rotation captures.

Mirrors fusion.c: a_center = a_imu - gain * (omega_dot x r + omega x (omega x r))

Usage:
  python test_spin_compensation.py
  python test_spin_compensation.py hand_spin_sim.jsonl motorSpinFinal.jsonl
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from pathlib import Path

from device_protocol import unpack_collar_wire_line
from lever_arm_comp import accel_magnitude, compensate_linear_accel
from lever_arm_config import CENTRIPETAL_GAIN_XYZ, IMU_LEVER_ARM_M
from sensor_stream import SensorStreamBuffer

MIN_SPIN_OMEGA_RAD_S = 0.5
DEFAULT_CAPTURES = (
    "hand_spin_sim.jsonl",
    "hand_spin_noisy.jsonl",
    "motorSpinFinal.jsonl",
    "capture1.jsonl",
)


def _integrate_stationarity(
    ticks: list[tuple[int, dict, dict]],
) -> dict[str, float]:
    """Rough velocity/position drift from compensated accel (spin samples only)."""
    vx = vy = vz = 0.0
    px = py = pz = 0.0
    prev_ts: int | None = None
    for ts_us, accel, _gyro in ticks:
        if prev_ts is None:
            prev_ts = ts_us
            continue
        dt = (ts_us - prev_ts) / 1_000_000.0
        prev_ts = ts_us
        if dt <= 0.0 or dt > 0.1:
            continue
        vx += accel["x"] * dt
        vy += accel["y"] * dt
        vz += accel["z"] * dt
        px += vx * dt
        py += vy * dt
        pz += vz * dt
    return {
        "velocity_m": math.sqrt(vx * vx + vy * vy + vz * vz),
        "position_m": math.sqrt(px * px + py * py + pz * pz),
    }


def analyze_capture(path: Path) -> dict[str, object]:
    raw_mags: list[float] = []
    comp_mags: list[float] = []
    spin_ticks: list[tuple[int, dict, dict]] = []
    prev_gyro: dict[str, float] | None = None
    prev_ts: int | None = None

    def on_tick(msg: dict) -> None:
        nonlocal prev_gyro, prev_ts
        gyro = msg.get("gyro")
        accel = msg.get("accel")
        if not gyro or not accel:
            return
        omega = math.sqrt(gyro["x"] ** 2 + gyro["y"] ** 2 + gyro["z"] ** 2)
        if omega < MIN_SPIN_OMEGA_RAD_S:
            return

        raw_mags.append(accel_magnitude(accel))
        dt_s = None
        if prev_ts is not None:
            dt_s = (msg["ts_us"] - prev_ts) / 1_000_000.0
        compensated = compensate_linear_accel(
            accel,
            gyro,
            prev_gyro=prev_gyro,
            dt_s=dt_s,
        )
        comp_mags.append(accel_magnitude(compensated))
        spin_ticks.append((msg["ts_us"], compensated, gyro))
        prev_gyro = dict(gyro)
        prev_ts = msg["ts_us"]

    buf = SensorStreamBuffer(on_tick=on_tick, output_hz=100, fixed_latency_us=50_000)
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("{"):
                continue
            samples = [
                (s.sensor, s.ts_us, s.data)
                for s in unpack_collar_wire_line(line)
            ]
            if samples:
                buf.ingest_sequence(samples)
    buf.flush()

    def _stats(vals: list[float]) -> dict[str, float]:
        if not vals:
            return {"n": 0.0}
        ordered = sorted(vals)
        return {
            "n": float(len(vals)),
            "median": statistics.median(vals),
            "p90": ordered[int(0.9 * len(ordered)) - 1],
            "max": max(vals),
        }

    drift = _integrate_stationarity(spin_ticks) if spin_ticks else {}

    return {
        "path": str(path),
        "spin_samples": len(spin_ticks),
        "raw": _stats(raw_mags),
        "compensated": _stats(comp_mags),
        "drift": drift,
        "lever_arm_m": dict(IMU_LEVER_ARM_M),
        "gain_xyz": CENTRIPETAL_GAIN_XYZ,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Test lever-arm spin compensation")
    parser.add_argument(
        "captures",
        nargs="*",
        default=list(DEFAULT_CAPTURES),
        help="JSONL capture paths",
    )
    args = parser.parse_args()

    print("Lever arm (center -> IMU):", IMU_LEVER_ARM_M)
    print("Centripetal gain XYZ:", CENTRIPETAL_GAIN_XYZ)
    print()

    for name in args.captures:
        path = Path(name)
        if not path.is_file():
            print(f"SKIP {path} (not found)")
            continue
        result = analyze_capture(path)
        raw = result["raw"]
        comp = result["compensated"]
        drift = result.get("drift") or {}
        print(f"=== {path.name} ===")
        print(f"  spin samples: {result['spin_samples']}")
        if raw.get("n", 0) > 0:
            print(
                f"  |accel| raw      median={raw['median']:.4f} m/s²  "
                f"p90={raw['p90']:.4f}"
            )
            print(
                f"  |accel| compensated median={comp['median']:.4f} m/s²  "
                f"p90={comp['p90']:.4f}"
            )
            if drift:
                print(
                    f"  naive drift (comp accel): vel~{drift.get('velocity_m', 0):.4f} m  "
                    f"pos~{drift.get('position_m', 0):.4f} m"
                )
        else:
            print("  no spin samples (|omega| too low)")
        print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
