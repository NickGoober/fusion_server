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

from fusion_calib import (
    default_calib_path,
    flow_lever_arm_from_calib,
    imu_lever_arm_from_calib,
    load_calib,
)


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


class FusionLeverArmCalResult(Structure):
    _fields_ = [
        ("success", c_bool),
        ("flow_lever_arm_m", FusionVec3),
        ("imu_lever_arm_m", FusionVec3),
        ("samples_used", c_uint32),
        ("samples_rejected", c_uint32),
        ("residual_rms_mps", c_float),
        ("axis", c_uint32),
        ("omega_rad_s", c_float),
    ]


class FusionLeverArmCalStatus(Structure):
    _fields_ = [
        ("active", c_bool),
        ("axis", c_uint32),
        ("axis_auto", c_bool),
        ("axis_locked", c_bool),
        ("expected_omega_rad_s", c_float),
        ("samples_used", c_uint32),
        ("samples_rejected", c_uint32),
    ]


CAL_AXIS_X = 0
CAL_AXIS_Y = 1
CAL_AXIS_Z = 2
CAL_AXIS_AUTO = 3

_AXIS_NAMES = ("x", "y", "z", "auto")


def parse_cal_axis(axis: str) -> int:
    axis = (axis or "auto").strip().lower()
    if axis == "auto":
        return CAL_AXIS_AUTO
    if axis == "y":
        return CAL_AXIS_Y
    if axis == "z":
        return CAL_AXIS_Z
    return CAL_AXIS_X


def cal_axis_name(axis: int) -> str:
    if 0 <= int(axis) < len(_AXIS_NAMES):
        return _AXIS_NAMES[int(axis)]
    return "unknown"


class FusionEngine:
    """Thin wrapper around fusion.c for sensor ingestion and pose output."""

    def __init__(
        self,
        lib_path: str | None = None,
        calib_path: str | Path | None = None,
    ) -> None:
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
        self._calib_path = Path(calib_path) if calib_path else default_calib_path()

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
            c_uint16, c_int64,
        ]
        self._lib.fusion_get_pose.argtypes = [POINTER(FusionPose)]

        self._lib.fusion_set_flow_lever_arm.argtypes = [c_float, c_float, c_float]
        self._lib.fusion_get_flow_lever_arm.argtypes = [POINTER(FusionVec3)]
        self._lib.fusion_set_imu_lever_arm.argtypes = [c_float, c_float, c_float]
        self._lib.fusion_get_imu_lever_arm.argtypes = [POINTER(FusionVec3)]

        self._lib.fusion_lever_arm_cal_start.argtypes = [c_uint32, c_float, c_float]
        self._lib.fusion_lever_arm_cal_start.restype = c_bool
        self._lib.fusion_lever_arm_cal_feed.argtypes = [
            c_float, c_float, c_float,
            c_float, c_float, c_float,
            c_int16, c_int16,
            c_uint16, c_float,
        ]
        self._lib.fusion_lever_arm_cal_feed.restype = c_bool
        self._lib.fusion_lever_arm_cal_finish.argtypes = [POINTER(FusionLeverArmCalResult)]
        self._lib.fusion_lever_arm_cal_finish.restype = c_bool
        self._lib.fusion_lever_arm_cal_cancel.restype = None
        self._lib.fusion_lever_arm_cal_get_status.argtypes = [POINTER(FusionLeverArmCalStatus)]

        calib = load_calib(self._calib_path)
        flow_arm = flow_lever_arm_from_calib(calib)
        imu_arm = imu_lever_arm_from_calib(calib)

        self._lib.fusion_set_debug_logging(False)
        if not self._lib.fusion_init():
            raise RuntimeError("fusion_init() failed")
        self._lib.fusion_set_flow_lever_arm(flow_arm[0], flow_arm[1], flow_arm[2])
        self._lib.fusion_set_imu_lever_arm(imu_arm[0], imu_arm[1], imu_arm[2])

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

    def submit_range(self, distance_mm: int, ts_us: int) -> None:
        self._lib.fusion_submit_range(distance_mm, ts_us)

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

    def get_flow_lever_arm(self) -> dict[str, float]:
        arm = FusionVec3()
        self._lib.fusion_get_flow_lever_arm(arm)
        return {"x": arm.x, "y": arm.y, "z": arm.z}

    def set_flow_lever_arm(self, x_m: float, y_m: float, z_m: float) -> None:
        self._lib.fusion_set_flow_lever_arm(x_m, y_m, z_m)

    def get_imu_lever_arm(self) -> dict[str, float]:
        arm = FusionVec3()
        self._lib.fusion_get_imu_lever_arm(arm)
        return {"x": arm.x, "y": arm.y, "z": arm.z}

    def set_imu_lever_arm(self, x_m: float, y_m: float, z_m: float) -> None:
        self._lib.fusion_set_imu_lever_arm(x_m, y_m, z_m)

    def lever_arm_cal_start(
        self,
        axis: str = "x",
        omega_rad_s: float = 0.0,
        omega_tol_rad_s: float = 0.0,
    ) -> bool:
        return bool(
            self._lib.fusion_lever_arm_cal_start(
                parse_cal_axis(axis),
                omega_rad_s,
                omega_tol_rad_s,
            )
        )

    def lever_arm_cal_feed(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        flow_dx: int,
        flow_dy: int,
        range_mm: int,
        dt_s: float = 0.01,
    ) -> bool:
        return bool(
            self._lib.fusion_lever_arm_cal_feed(
                gx, gy, gz, ax, ay, az, flow_dx, flow_dy, range_mm, dt_s,
            )
        )

    def lever_arm_cal_finish(self) -> dict | None:
        result = FusionLeverArmCalResult()
        if not self._lib.fusion_lever_arm_cal_finish(result):
            return None
        return {
            "success": bool(result.success),
            "flow_lever_arm_m": {
                "x": result.flow_lever_arm_m.x,
                "y": result.flow_lever_arm_m.y,
                "z": result.flow_lever_arm_m.z,
            },
            "imu_lever_arm_m": {
                "x": result.imu_lever_arm_m.x,
                "y": result.imu_lever_arm_m.y,
                "z": result.imu_lever_arm_m.z,
            },
            "samples_used": int(result.samples_used),
            "samples_rejected": int(result.samples_rejected),
            "residual_rms_mps": float(result.residual_rms_mps),
            "axis": int(result.axis),
            "omega_rad_s": float(result.omega_rad_s),
        }

    def lever_arm_cal_cancel(self) -> None:
        self._lib.fusion_lever_arm_cal_cancel()

    def lever_arm_cal_status(self) -> dict:
        status = FusionLeverArmCalStatus()
        self._lib.fusion_lever_arm_cal_get_status(status)
        axis = int(status.axis)
        return {
            "active": bool(status.active),
            "axis": axis,
            "axis_name": cal_axis_name(axis if not status.axis_auto or status.axis_locked else CAL_AXIS_AUTO),
            "detected_axis": cal_axis_name(axis) if status.axis_locked else None,
            "axis_auto": bool(status.axis_auto),
            "axis_locked": bool(status.axis_locked),
            "expected_omega_rad_s": float(status.expected_omega_rad_s),
            "samples_used": int(status.samples_used),
            "samples_rejected": int(status.samples_rejected),
        }

    @property
    def calib_path(self) -> Path:
        return self._calib_path
