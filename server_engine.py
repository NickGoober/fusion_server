"""Shared fusion engine singleton for all collar sessions."""

from __future__ import annotations

import logging
import threading
from typing import Any, Callable, TypeVar

from fusion_lib import FusionEngine

LOG = logging.getLogger("fusion_server.engine")

T = TypeVar("T")

_engine: FusionEngine | None = None
_engine_lock = threading.Lock()
_cal_meta: dict[str, Any] = {"axis": "auto", "omega_rad_s": 0.0}


def get_fusion_engine() -> FusionEngine:
    global _engine
    with _engine_lock:
        if _engine is None:
            _engine = FusionEngine()
            imu_arm = _engine.get_imu_lever_arm()
            if _engine.imu_only:
                LOG.info(
                    "Fusion engine ready (IMU-only mode, imu lever arm: "
                    "x=%.4f y=%.4f z=%.4f m)",
                    imu_arm["x"], imu_arm["y"], imu_arm["z"],
                )
            else:
                arm = _engine.get_flow_lever_arm()
                LOG.info(
                    "Fusion engine ready (flow lever arm: x=%.4f y=%.4f z=%.4f m, "
                    "imu lever arm: x=%.4f y=%.4f z=%.4f m)",
                    arm["x"], arm["y"], arm["z"],
                    imu_arm["x"], imu_arm["y"], imu_arm["z"],
                )
        return _engine


def with_engine(fn: Callable[[], T]) -> T:
    with _engine_lock:
        return fn()


def get_cal_meta() -> dict[str, Any]:
    return _cal_meta


def set_cal_meta(*, axis: str, omega_rad_s: float) -> None:
    _cal_meta["axis"] = axis
    _cal_meta["omega_rad_s"] = omega_rad_s
