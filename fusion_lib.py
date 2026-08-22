"""ctypes bridge to the vendored Raedir fusion EKF (libfusion.so)."""

from __future__ import annotations

import time
from ctypes import CDLL, c_bool, c_float, c_int, c_int16, c_int64, c_uint8, c_uint16, c_uint32, POINTER, Structure
from pathlib import Path

from fusion_settings import get_bool_setting, get_float_setting, get_setting
from lever_arm_config import CENTRIPETAL_GAIN_XYZ, IMU_LEVER_ARM_M


def _fusion_sensor_flags() -> tuple[bool, bool]:
    """Return (use_optical_flow, use_range) honoring IMU_ONLY_MODE."""
    imu_only = get_bool_setting("IMU_ONLY_MODE", True)
    if imu_only:
        return False, False
    use_flow = get_bool_setting("FUSION_USE_OPTICAL_FLOW", True)
    use_range = get_bool_setting("FUSION_USE_RANGE", True)
    return use_flow, use_range


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
        ("linear_accel_mps2", FusionVec3),
        ("valid", c_bool),
    ]


class FusionEngine:
    """Thin wrapper around fusion.c for sensor ingestion and pose output."""

    def __init__(self, lib_path: str | None = None) -> None:
        if lib_path is None:
            lib_path = get_setting(
                "FUSION_LIB_PATH",
                str(Path(__file__).resolve().parent / "native" / "libfusion.so"),
            )
        path = Path(lib_path)
        if not path.is_file():
            raise FileNotFoundError(
                f"Fusion library not found at {path}. "
                f"On the server run: cd {path.parent.parent} && ./build_lib.sh"
            )

        try:
            self._lib = CDLL(str(path))
        except OSError as exc:
            hint = ""
            msg = str(exc).lower()
            if path.stat().st_size == 0:
                hint = " File is empty — run ./build_lib.sh on this machine."
            elif "invalid elf header" in msg or "wrong elf class" in msg:
                hint = (
                    " File is not a Linux shared library for this host — "
                    "run ./build_lib.sh here (do not copy .so from Windows)."
                )
            raise OSError(
                f"Cannot load fusion library at {path}: {exc}.{hint}"
            ) from exc

        self._lib.fusion_init.restype = c_bool
        self._lib.fusion_init_imu_only.restype = c_bool
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
        self._lib.fusion_set_imu_centripetal_gain.argtypes = [c_float, c_float, c_float]
        self._lib.fusion_set_imu_to_body.argtypes = [c_float, c_float, c_float, c_float]
        self._lib.fusion_get_imu_to_body.argtypes = [POINTER(FusionQuat)]
        if hasattr(self._lib, "fusion_set_imu_accel_mode"):
            self._lib.fusion_set_imu_accel_mode.argtypes = [c_int]
            self._lib.fusion_set_imu_accel_mode.restype = None
        if hasattr(self._lib, "fusion_set_world_gravity"):
            self._lib.fusion_set_world_gravity.argtypes = [c_float, c_float, c_float]
            self._lib.fusion_set_world_gravity.restype = None
        if hasattr(self._lib, "fusion_set_quat_filter"):
            self._lib.fusion_set_quat_filter.argtypes = [c_bool, c_float, c_float]
            self._lib.fusion_set_quat_filter.restype = None
        if hasattr(self._lib, "fusion_set_require_flow"):
            self._lib.fusion_set_require_flow.argtypes = [c_bool]
            self._lib.fusion_set_require_flow.restype = None
        if hasattr(self._lib, "fusion_set_require_range"):
            self._lib.fusion_set_require_range.argtypes = [c_bool]
            self._lib.fusion_set_require_range.restype = None

        self._lib.fusion_set_debug_logging(False)
        use_flow, use_range = _fusion_sensor_flags()
        self.use_optical_flow = use_flow
        self.use_range = use_range
        imu_only = not use_flow and not use_range
        self.imu_only = imu_only
        if imu_only:
            if not self._lib.fusion_init_imu_only():
                raise RuntimeError("fusion_init_imu_only() failed")
        elif not self._lib.fusion_init():
            raise RuntimeError("fusion_init() failed")
        elif hasattr(self._lib, "fusion_set_require_flow"):
            self._lib.fusion_set_require_flow(use_flow)
            self._lib.fusion_set_require_range(use_range)

        self._lib.fusion_set_imu_lever_arm(
            IMU_LEVER_ARM_M["x"],
            IMU_LEVER_ARM_M["y"],
            IMU_LEVER_ARM_M["z"],
        )
        if hasattr(self._lib, "fusion_set_imu_centripetal_gain"):
            self._lib.fusion_set_imu_centripetal_gain(
                CENTRIPETAL_GAIN_XYZ[0],
                CENTRIPETAL_GAIN_XYZ[1],
                CENTRIPETAL_GAIN_XYZ[2],
            )
        self._configure_imu_pipeline()
        self._lib.fusion_set_imu_to_body(1.0, 0.0, 0.0, 0.0)

        deadline = time.monotonic() + 5.0
        while not self._lib.fusion_is_ready():
            if time.monotonic() > deadline:
                raise RuntimeError("fusion_is_ready() timed out")
            time.sleep(0.001)

    def reset(self) -> None:
        self._lib.fusion_reset()

    def _configure_imu_pipeline(self) -> None:
        """Wire quat smoothing + gravity-vector accel mode from server settings."""
        accel_mode = (get_setting("IMU_ACCEL_MODE", "gravity_vector") or "gravity_vector").strip().lower()
        mode_map = {
            "linear": 0,
            "specific_force": 1,
            "specific": 1,
            "accel": 1,
            "gravity_vector": 2,
            "gravity": 2,
        }
        if hasattr(self._lib, "fusion_set_imu_accel_mode"):
            self._lib.fusion_set_imu_accel_mode(mode_map.get(accel_mode, 2))

        gx = get_float_setting("WORLD_GRAVITY_X", 0.0)
        gy = get_float_setting("WORLD_GRAVITY_Y", -9.81)
        gz = get_float_setting("WORLD_GRAVITY_Z", 0.0)
        if hasattr(self._lib, "fusion_set_world_gravity"):
            self._lib.fusion_set_world_gravity(gx, gy, gz)

        quat_filter = get_bool_setting("QUAT_FILTER_ENABLE", True)
        quat_tau = get_float_setting("QUAT_FILTER_TAU_S", 0.04)
        quat_max_step = get_float_setting("QUAT_FILTER_MAX_STEP_RAD", 0.12)
        if hasattr(self._lib, "fusion_set_quat_filter"):
            self._lib.fusion_set_quat_filter(quat_filter, quat_tau, quat_max_step)

    def submit_quat(self, w: float, x: float, y: float, z: float, ts_us: int) -> None:
        self._lib.fusion_submit_imu_quat(w, x, y, z, ts_us)

    def submit_gyro(self, gx: float, gy: float, gz: float, ts_us: int) -> None:
        self._lib.fusion_submit_imu_gyro(gx, gy, gz, ts_us)

    def submit_accel(self, ax: float, ay: float, az: float, ts_us: int) -> None:
        self._lib.fusion_submit_imu_accel(ax, ay, az, ts_us)

    def submit_flow(
        self, dx: int, dy: int, quality: int, ts_us: int,
    ) -> None:
        if not self.use_optical_flow:
            return
        self._lib.fusion_submit_flow(dx, dy, quality, ts_us)

    def submit_range(self, distance_mm: int, ts_us: int) -> None:
        if not self.use_range:
            return
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
            "linear_accel_mps2": {
                "x": pose.linear_accel_mps2.x,
                "y": pose.linear_accel_mps2.y,
                "z": pose.linear_accel_mps2.z,
            },
            "valid": bool(pose.valid),
        }

    def get_flow_lever_arm(self) -> dict[str, float]:
        arm = FusionVec3()
        self._lib.fusion_get_flow_lever_arm(arm)
        return {"x": arm.x, "y": arm.y, "z": arm.z}

    def get_imu_lever_arm(self) -> dict[str, float]:
        arm = FusionVec3()
        self._lib.fusion_get_imu_lever_arm(arm)
        return {"x": arm.x, "y": arm.y, "z": arm.z}

    def get_imu_to_body(self) -> dict[str, float]:
        quat = FusionQuat()
        self._lib.fusion_get_imu_to_body(quat)
        return {"w": quat.w, "x": quat.x, "y": quat.y, "z": quat.z}
