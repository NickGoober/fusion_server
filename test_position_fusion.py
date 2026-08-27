#!/usr/bin/env python3
"""Unit + capture replay tests for dual-channel barbell position fusion."""

from __future__ import annotations

import json
import math
import unittest
from pathlib import Path

from direct_position import DirectPositionTracker
from position_fusion import PositionFusionEngine, PositionKalmanFilter
from sensor_stream import (
    SENSOR_ACCEL,
    SENSOR_FLOW,
    SENSOR_QUAT,
    SENSOR_RADAR,
    payload_array_to_dict,
)

IDENTITY_Q = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
# BNO chip Z-up → collar Y-up (−90 deg about +X), same as fusion_lib default.
MOUNT_Q = {"w": 0.7071067811865476, "x": -0.7071067811865476, "y": 0.0, "z": 0.0}
CAPTURE = Path(__file__).resolve().parent / "captures" / "freeMoveLR_flow_fixed.jsonl"


def _rms_steps(xs: list[float]) -> float:
    if len(xs) < 2:
        return 0.0
    acc = 0.0
    n = 0
    for a, b in zip(xs, xs[1:]):
        d = b - a
        acc += d * d
        n += 1
    return math.sqrt(acc / n) if n else 0.0


def _corr(xs: list[float], ys: list[float]) -> float:
    n = len(xs)
    if n < 3:
        return 1.0
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((a - mx) * (b - my) for a, b in zip(xs, ys))
    dx = math.sqrt(sum((a - mx) ** 2 for a in xs))
    dy = math.sqrt(sum((b - my) ** 2 for b in ys))
    if dx < 1e-12 or dy < 1e-12:
        return 1.0
    return num / (dx * dy)


class KalmanUnitTests(unittest.TestCase):
    def test_predict_constant_velocity(self) -> None:
        kf = PositionKalmanFilter()
        kf.x[3] = 1.0
        pxx = kf.P[0][0]
        kf.predict(0.1)
        self.assertAlmostEqual(kf.x[0], 0.1, places=9)
        self.assertGreater(kf.P[0][0], pxx)
        self.assertGreater(kf.P[3][3], 0.0)
        kf.predict(0.1, coast_xz=False)
        self.assertAlmostEqual(kf.x[0], 0.1, places=9)
        kf.x[5] = 0.5  # vy does coast into y so rejected radar can interpolate
        y0 = kf.x[2]
        kf.predict(0.1)
        self.assertAlmostEqual(kf.x[2], y0 + 0.05, places=9)

    def test_update_y_tracks_measurement(self) -> None:
        kf = PositionKalmanFilter(range_std_m=0.003)
        kf.update_y(0.0, coupling=1.0)
        for _ in range(40):
            kf.predict(0.01)
            kf.update_y(0.12, coupling=1.0)
        self.assertGreater(kf.position()["y"], 0.05)
        self.assertLess(abs(kf.position()["y"] - 0.12), 0.03)

    def test_innovation_gate_rejects_flow_spike(self) -> None:
        kf = PositionKalmanFilter(innovation_gate_sigma=3.0, flow_std_base_m=0.002)
        kf.seed_y(0.0)
        for _ in range(20):
            kf.predict(0.01)
            pos = kf.position()
            kf.update_xz_increment(
                pos["x"] + 0.001,
                pos["z"],
                0.001,
                0.0,
                0.01,
                height_m=0.6,
                quality=255,
                coupling=1.0,
                fov_deg=42.0,
                npix=35.0,
                max_pixels=40,
                dx_px=1,
                dy_px=0,
            )
        before = kf.position()["x"]
        ok = kf.update_xz_increment(
            before + 0.4,
            0.0,
            0.4,
            0.0,
            0.01,
            height_m=0.6,
            quality=255,
            coupling=1.0,
            fov_deg=42.0,
            npix=35.0,
            max_pixels=40,
            dx_px=8,
            dy_px=0,
        )
        self.assertFalse(ok)
        self.assertTrue(kf.last_reject)
        self.assertLess(abs(kf.position()["x"] - before), 0.05)

    def test_pixel_spike_gate(self) -> None:
        kf = PositionKalmanFilter()
        kf.seed_y(0.0)
        ok = kf.update_xz_increment(
            0.2,
            0.0,
            0.2,
            0.0,
            0.01,
            height_m=0.6,
            quality=255,
            coupling=1.0,
            fov_deg=42.0,
            npix=35.0,
            max_pixels=40,
            dx_px=80,
            dy_px=0,
        )
        self.assertFalse(ok)
        self.assertEqual(kf.position()["x"], 0.0)


class EngineTests(unittest.TestCase):
    def test_kalman_disabled_matches_raw(self) -> None:
        engine = PositionFusionEngine(kalman_enable=False)
        tracker = DirectPositionTracker()
        ts = 0
        for dx in (0, 3, -1, 4, 2, -2, 5):
            ts += 10_000
            kwargs = dict(
                range_mm=600,
                flow={"dx": dx, "dy": 0, "quality": 200} if dx else None,
                imu_quat=IDENTITY_Q,
                imu_to_body=MOUNT_Q,
            )
            engine.update(ts_us=ts, **kwargs)
            tracker.update(**kwargs)
            self.assertEqual(engine.raw_position(), tracker.position())
            self.assertEqual(engine.filtered_position(), engine.raw_position())
            self.assertEqual(engine.filtered_velocity(), {"x": 0.0, "y": 0.0, "z": 0.0})

    def test_filtered_smooths_vibration_raw_keeps_impulses(self) -> None:
        engine = PositionFusionEngine(kalman_enable=True, process_noise_vel=0.5)
        raw_x: list[float] = []
        filt_x: list[float] = []
        ts = 0
        engine.update(
            range_mm=550,
            flow=None,
            imu_quat=IDENTITY_Q,
            imu_to_body=MOUNT_Q,
            ts_us=ts,
        )
        # Alternating ±8 px (vibration) plus a single 25 px impulse.
        pattern = [8, -8, 8, -8, 8, -8, 25, -8, 8, -8, 8, -8, 8, -8]
        for dx in pattern:
            ts += 10_000
            engine.update(
                range_mm=550,
                flow={"dx": dx, "dy": 0, "quality": 255},
                imu_quat=IDENTITY_Q,
                imu_to_body=MOUNT_Q,
                ts_us=ts,
            )
            raw_x.append(engine.raw_position()["x"])
            filt_x.append(engine.filtered_position()["x"])

        # Impulse: raw jumps more than filtered on the 25 px frame (index 6).
        raw_jump = abs(raw_x[6] - raw_x[5])
        filt_jump = abs(filt_x[6] - filt_x[5])
        self.assertGreater(raw_jump, 0.0)
        self.assertLess(filt_jump, raw_jump)

        self.assertLess(_rms_steps(filt_x), _rms_steps(raw_x))
        # Filtered still follows the same gross direction as raw.
        self.assertEqual(math.copysign(1.0, filt_x[-1] or 1.0), math.copysign(1.0, raw_x[-1] or 1.0))
        self.assertGreater(abs(filt_x[-1]), 0.25 * abs(raw_x[-1]) if raw_x[-1] else 0.0)

    def _radar_tick(self, engine: PositionFusionEngine, ts: int, range_mm: int) -> None:
        engine.update(
            range_mm=range_mm,
            flow=None,
            imu_quat=IDENTITY_Q,
            imu_to_body=MOUNT_Q,
            ts_us=ts,
            radar_update=True,
        )

    def test_radar_one_frame_spike_rejected_raw_keeps_it(self) -> None:
        engine = PositionFusionEngine(kalman_enable=True)
        ts = 0
        for _ in range(8):
            ts += 20_000
            self._radar_tick(engine, ts, 600)
        y_hold = engine.filtered_position()["y"]
        ts += 20_000
        self._radar_tick(engine, ts, 720)  # +120 mm glitch
        self.assertGreater(abs(engine.raw_position()["y"] - y_hold), 0.08)
        self.assertLess(abs(engine.filtered_position()["y"] - y_hold), 0.03)
        ts += 20_000
        self._radar_tick(engine, ts, 600)  # snap back
        self.assertLess(abs(engine.filtered_position()["y"] - y_hold), 0.03)

    def test_radar_two_frame_spike_then_snap_back(self) -> None:
        engine = PositionFusionEngine(kalman_enable=True)
        ts = 0
        for _ in range(8):
            ts += 20_000
            self._radar_tick(engine, ts, 550)
        y_hold = engine.filtered_position()["y"]
        ts += 20_000
        self._radar_tick(engine, ts, 680)
        raw_spike = engine.raw_position()["y"]
        ts += 20_000
        self._radar_tick(engine, ts, 675)
        self.assertGreater(abs(raw_spike - y_hold), 0.08)
        self.assertLess(abs(engine.filtered_position()["y"] - y_hold), 0.04)
        ts += 20_000
        self._radar_tick(engine, ts, 550)
        self.assertLess(abs(engine.filtered_position()["y"] - y_hold), 0.04)

    def test_radar_two_frame_spike_rejected_even_with_streak_two(self) -> None:
        engine = PositionFusionEngine(kalman_enable=True, radar_max_reject_streak=2)
        ts = 0
        for _ in range(8):
            ts += 20_000
            self._radar_tick(engine, ts, 660)
        y_hold = engine.filtered_position()["y"]
        ts += 20_000
        self._radar_tick(engine, ts, 129)
        ts += 22_000
        self._radar_tick(engine, ts, 129)
        self.assertLess(abs(engine.filtered_position()["y"] - y_hold), 0.04)
        ts += 23_000
        self._radar_tick(engine, ts, 655)
        self.assertLess(abs(engine.filtered_position()["y"] - y_hold), 0.05)

    def test_flow_zeros_interpolate_instead_of_stutter(self) -> None:
        engine = PositionFusionEngine(kalman_enable=True)
        ts = 0
        engine.update(
            range_mm=600,
            flow=None,
            imu_quat=IDENTITY_Q,
            imu_to_body=MOUNT_Q,
            ts_us=ts,
            radar_update=True,
        )
        raw_x: list[float] = []
        filt_x: list[float] = []
        for i in range(20):
            ts += 10_000
            flow = {"dx": 4, "dy": 0, "quality": 200} if i % 2 == 0 else {
                "dx": 0, "dy": 0, "quality": 200,
            }
            engine.update(
                range_mm=600,
                flow=flow,
                imu_quat=IDENTITY_Q,
                imu_to_body=MOUNT_Q,
                ts_us=ts,
                radar_update=False,
            )
            raw_x.append(engine.raw_position()["x"])
            filt_x.append(engine.filtered_position()["x"])
        raw_steps = [abs(b - a) for a, b in zip(raw_x, raw_x[1:])]
        filt_steps = [abs(b - a) for a, b in zip(filt_x, filt_x[1:])]
        self.assertGreater(max(raw_steps), max(filt_steps))
        self.assertGreater(abs(filt_x[-1]), 0.25 * abs(raw_x[-1]))

    def test_radar_sustained_lift_is_not_frozen(self) -> None:
        engine = PositionFusionEngine(kalman_enable=True)
        ts = 0
        mm = 800
        for _ in range(6):
            ts += 20_000
            self._radar_tick(engine, ts, mm)
        for _ in range(12):
            ts += 20_000
            mm += 20  # radar looks down: larger range = bar higher
            self._radar_tick(engine, ts, mm)
        self.assertGreater(engine.filtered_position()["y"], 0.12)
        self.assertLess(abs(engine.filtered_position()["y"] - engine.raw_position()["y"]), 0.08)


def _iter_capture(path: Path):
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if isinstance(obj, dict):
                continue
            rows = obj if obj and isinstance(obj[0], list) else [obj]
            for row in rows:
                if not isinstance(row, list) or len(row) < 3:
                    continue
                parsed = payload_array_to_dict(int(row[0]), row[2])
                if parsed is None:
                    continue
                sensor, data = parsed
                yield int(row[1]), sensor, data


def _replay_into(update, *, use_ts: bool) -> None:
    last_quat: dict[str, float] | None = None
    last_range: int | None = None
    for ts, sensor, data in _iter_capture(CAPTURE):
        if sensor == SENSOR_QUAT:
            last_quat = {
                "w": float(data["w"]),
                "x": float(data["x"]),
                "y": float(data["y"]),
                "z": float(data["z"]),
            }
        elif sensor == SENSOR_RADAR:
            last_range = int(data["mm"])
        elif sensor == SENSOR_ACCEL:
            pass
        flow = data if sensor == SENSOR_FLOW else None
        kwargs = dict(
            range_mm=last_range,
            flow=flow,
            imu_quat=last_quat,
            imu_to_body=MOUNT_Q,
            radar_update=(sensor == SENSOR_RADAR),
        )
        if use_ts:
            update(ts_us=ts, **kwargs)
        else:
            update(**kwargs)


@unittest.skipUnless(CAPTURE.is_file(), "missing captures/freeMoveLR_flow_fixed.jsonl")
class CaptureReplayTests(unittest.TestCase):
    def test_raw_matches_direct_tracker(self) -> None:
        tracker = DirectPositionTracker()
        engine = PositionFusionEngine(kalman_enable=True)
        last_quat = None
        last_range = None
        n = 0
        for ts, sensor, data in _iter_capture(CAPTURE):
            if sensor == SENSOR_QUAT:
                last_quat = {
                    "w": float(data["w"]),
                    "x": float(data["x"]),
                    "y": float(data["y"]),
                    "z": float(data["z"]),
                }
            elif sensor == SENSOR_RADAR:
                last_range = int(data["mm"])
            flow = data if sensor == SENSOR_FLOW else None
            kwargs = dict(
                range_mm=last_range,
                flow=flow,
                imu_quat=last_quat,
                imu_to_body=MOUNT_Q,
            )
            tracker.update(**kwargs)
            engine.update(ts_us=ts, **kwargs)
            rp = engine.raw_position()
            tp = tracker.position()
            self.assertAlmostEqual(rp["x"], tp["x"], places=9)
            self.assertAlmostEqual(rp["y"], tp["y"], places=9)
            self.assertAlmostEqual(rp["z"], tp["z"], places=9)
            n += 1
        self.assertGreater(n, 100)

    def test_filtered_tracks_gross_motion_with_less_hf(self) -> None:
        engine = PositionFusionEngine(kalman_enable=True)
        _replay_into(engine.update, use_ts=True)
        raw = engine.raw_position()
        filt = engine.filtered_position()
        # Same ballpark as the capture path (left/right move).
        self.assertGreater(max(abs(raw["x"]), abs(raw["z"])), 0.05)
        self.assertLess(math.hypot(filt["x"] - raw["x"], filt["z"] - raw["z"]), 0.35)
        self.assertLess(abs(filt["y"] - raw["y"]), 0.08)

        raw_xs: list[float] = []
        filt_xs: list[float] = []
        engine.reset()
        last_quat = None
        last_range = None
        for ts, sensor, data in _iter_capture(CAPTURE):
            if sensor == SENSOR_QUAT:
                last_quat = {
                    "w": float(data["w"]),
                    "x": float(data["x"]),
                    "y": float(data["y"]),
                    "z": float(data["z"]),
                }
            elif sensor == SENSOR_RADAR:
                last_range = int(data["mm"])
            flow = data if sensor == SENSOR_FLOW else None
            engine.update(
                range_mm=last_range,
                flow=flow,
                imu_quat=last_quat,
                imu_to_body=MOUNT_Q,
                ts_us=ts,
                radar_update=(sensor == SENSOR_RADAR),
            )
            if sensor == SENSOR_FLOW:
                raw_xs.append(engine.raw_position()["x"])
                filt_xs.append(engine.filtered_position()["x"])
        self.assertGreater(_corr(filt_xs, raw_xs), 0.85)

    def test_freemove_lr_radar_glitches_at_4p9_and_10p6(self) -> None:
        engine = PositionFusionEngine(kalman_enable=True, radar_max_reject_streak=2)
        last_quat = None
        last_range = None
        t0 = None
        prev_fy = None
        max_jump_4p9 = 0.0
        max_jump_10p6 = 0.0
        for ts, sensor, data in _iter_capture(CAPTURE):
            if t0 is None:
                t0 = ts
            if sensor == SENSOR_QUAT:
                last_quat = {
                    "w": float(data["w"]),
                    "x": float(data["x"]),
                    "y": float(data["y"]),
                    "z": float(data["z"]),
                }
            elif sensor == SENSOR_RADAR:
                last_range = int(data["mm"])
            flow = data if sensor == SENSOR_FLOW else None
            fy_before = engine.filtered_position()["y"]
            engine.update(
                range_mm=last_range,
                flow=flow,
                imu_quat=last_quat,
                imu_to_body=MOUNT_Q,
                ts_us=ts,
                radar_update=(sensor == SENSOR_RADAR),
            )
            if sensor != SENSOR_RADAR:
                continue
            sec = (ts - t0) / 1e6
            jump = abs(engine.filtered_position()["y"] - fy_before)
            if 4.85 <= sec <= 5.10:
                max_jump_4p9 = max(max_jump_4p9, jump)
            if 10.50 <= sec <= 10.75:
                max_jump_10p6 = max(max_jump_10p6, jump)
            prev_fy = engine.filtered_position()["y"]
        self.assertLess(max_jump_4p9, 0.03)
        self.assertLess(max_jump_10p6, 0.03)
        _ = prev_fy


if __name__ == "__main__":
    unittest.main()
