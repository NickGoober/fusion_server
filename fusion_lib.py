"""ctypes bridge to the vendored Raedir fusion EKF (libfusion.so)."""

from __future__ import annotations

import os
import time
from ctypes import (
    CDLL,
    c_bool,
    c_float,
    c_int16,
    c_int64,
    c_uint16,
    c_uint32,
    c_uint8,
    POINTER,
    Structure,
)
from pathlib import Path


class FusionVec3(Structure):
    _fields_ = [("x", c_float), ("y", c_float), ("z", c_float)]


class FusionQuat(Structure):
    _fields_ = [("w", c_float), ("x", c_float), ("y", c_float), ("z", c_float)]


class FusionPose(Structure):
    _fields_ = [
        ("timestamp_us", c_int64),
        ("step_count", c_uint32),
        ("position_m", FusionVec3),
        ("velocity_mps", FusionVec3),
        ("rotation", FusionQuat),
        ("rotation_vector_rad", FusionVec3),
        ("euler_rpy_rad", FusionVec3),
        ("valid", c_bool),
    ]


class FusionEngine:
    """Thin wrapper around fusion.c for sensor ingestion and pose output."""

    def __init__(self, lib_path: str | None = None) -> None:
        if lib_path is None:
            lib_path = os.environ.get(
                "FUSION_LIB_PATH",
                str(Path(__file__).resolve().parent / "native" / "libfusion.so"),
            )
        path = Path(lib_path)
        if not path.is_file():
            raise FileNotFoundError(
                f"Fusion library not found at {path}. Run ./build_lib.sh first."
            )

        self._lib = CDLL(str(path))

        self._lib.fusion_init.restype = c_bool
        self._lib.fusion_reset.restype = None
        self._lib.fusion_is_ready.restype = c_bool
        self._lib.fusion_get_pose.restype = c_bool
        self._lib.fusion_set_debug_logging.restype = None

        self._lib.fusion_submit_imu_quat.argtypes = [
            c_float, c_float, c_float, c_float, c_int64,
        ]
        self._lib.fusion_submit_imu_gyro.argtypes = [
            c_float, c_float, c_float, c_int64,
        ]
        self._lib.fusion_submit_imu_accel.argtypes = [
            c_float, c_float, c_float, c_int64,
        ]
        self._lib.fusion_submit_flow.argtypes = [
            c_int16, c_int16, c_uint8, c_int64,
        ]
        self._lib.fusion_submit_range.argtypes = [
            c_uint16, c_int16, c_int64,
        ]
        self._lib.fusion_get_pose.argtypes = [POINTER(FusionPose)]

        self._lib.fusion_set_debug_logging(False)
        if not self._lib.fusion_init():
            raise RuntimeError("fusion_init() failed")

        deadline = time.monotonic() + 5.0
        while not self._lib.fusion_is_ready():
            if time.monotonic() > deadline:
                raise RuntimeError("fusion_is_ready() timed out")
            time.sleep(0.001)

    def reset(self) -> None:
        self._lib.fusion_reset()

    def submit_quat(self, w: float, x: float, y: float, z: float, ts_us: int) -> None:
        self._lib.fusion_submit_imu_quat(w, x, y, z, ts_us)

    def submit_gyro(self, gx: float, gy: float, gz: float, ts_us: int) -> None:
        self._lib.fusion_submit_imu_gyro(gx, gy, gz, ts_us)

    def submit_accel(self, ax: float, ay: float, az: float, ts_us: int) -> None:
        self._lib.fusion_submit_imu_accel(ax, ay, az, ts_us)

    def submit_flow(
        self, dx: int, dy: int, quality: int, ts_us: int,
    ) -> None:
        self._lib.fusion_submit_flow(dx, dy, quality, ts_us)

    def submit_range(self, distance_mm: int, strength: int, ts_us: int) -> None:
        self._lib.fusion_submit_range(distance_mm, strength, ts_us)

    def get_pose(self) -> dict | None:
        pose = FusionPose()
        if not self._lib.fusion_get_pose(pose):
            return None
        return {
            "timestamp_us": int(pose.timestamp_us),
            "step_count": int(pose.step_count),
            "position_m": {
                "x": pose.position_m.x,
                "y": pose.position_m.y,
                "z": pose.position_m.z,
            },
            "velocity_mps": {
                "x": pose.velocity_mps.x,
                "y": pose.velocity_mps.y,
                "z": pose.velocity_mps.z,
            },
            "rotation": {
                "w": pose.rotation.w,
                "x": pose.rotation.x,
                "y": pose.rotation.y,
                "z": pose.rotation.z,
            },
            "rotation_vector_rad": {
                "x": pose.rotation_vector_rad.x,
                "y": pose.rotation_vector_rad.y,
                "z": pose.rotation_vector_rad.z,
            },
            "euler_rpy_rad": {
                "x": pose.euler_rpy_rad.x,
                "y": pose.euler_rpy_rad.y,
                "z": pose.euler_rpy_rad.z,
            },
            "valid": bool(pose.valid),
        }
