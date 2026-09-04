#!/usr/bin/env python3
"""Compact app-stream encoding: Y/Z swap and minimal fields."""

from __future__ import annotations

import json
import math
import unittest

from pose_stream_format import compact_app_frame, swap_yz_quat, swap_yz_vec


class PoseStreamFormatTests(unittest.TestCase):
    def test_position_swaps_y_and_z(self) -> None:
        self.assertEqual(swap_yz_vec({"x": 0.12, "y": 0.01, "z": 0.04}), [0.12, 0.04, 0.01])

    def test_identity_quat_stays_identity(self) -> None:
        self.assertEqual(swap_yz_quat({"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}), [1.0, 0.0, 0.0, 0.0])

    def test_90deg_about_x_maps_up_to_forward(self) -> None:
        # Internal +90° about +X takes +Y (up) to +Z (forward).
        h = math.sqrt(0.5)
        q = swap_yz_quat({"w": h, "x": h, "y": 0.0, "z": 0.0})
        self.assertAlmostEqual(q[0], h, places=4)
        self.assertAlmostEqual(q[1], -h, places=4)
        self.assertAlmostEqual(q[2], 0.0, places=4)
        self.assertAlmostEqual(q[3], 0.0, places=4)

    def test_compact_frame_keys_and_arrays(self) -> None:
        payload = {
            "streaming": True,
            "frame_seq": 7,
            "updated_at_ms": 1000,
            "floor_offset_m": 0.6543,
            "pose": {
                "timestamp_us": 1234567890123,
                "position_m": {"x": 0.12, "y": 0.01, "z": 0.04},
                "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            },
            "pose_raw": {
                "position_m": {"x": 0.13, "y": -0.02, "z": 0.05},
            },
        }
        frame = compact_app_frame(payload)
        self.assertEqual(set(frame), {"t", "f", "n", "s", "p", "r"})
        self.assertEqual(frame["t"], 1234567890123)
        self.assertEqual(frame["f"], 0.654)
        self.assertEqual(frame["n"], 7)
        self.assertEqual(frame["s"], 1)
        self.assertEqual(frame["p"], [0.12, 0.04, 0.01, 0.13, 0.05, -0.02])
        self.assertEqual(frame["r"], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        encoded = json.dumps(frame, separators=(",", ":"))
        self.assertNotIn("position_m", encoded)
        self.assertNotIn("session_id", encoded)
        self.assertLess(len(encoded), 160)


if __name__ == "__main__":
    unittest.main()
