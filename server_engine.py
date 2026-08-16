"""Shared fusion engine singleton for all collar sessions."""

from __future__ import annotations

import logging
import threading
from typing import Callable, TypeVar

from fusion_lib import FusionEngine
from lever_arm_config import IMU_LEVER_ARM_M

LOG = logging.getLogger("fusion_server.engine")

T = TypeVar("T")

_engine: FusionEngine | None = None
_engine_lock = threading.Lock()


def get_fusion_engine() -> FusionEngine:
    global _engine
    with _engine_lock:
        if _engine is None:
            _engine = FusionEngine()
            imu_arm = _engine.get_imu_lever_arm()
            LOG.info(
                "Fusion engine ready (IMU lever arm x=%.4f y=%.4f z=%.4f m)",
                imu_arm["x"],
                imu_arm["y"],
                imu_arm["z"],
            )
            if imu_arm != IMU_LEVER_ARM_M:
                LOG.warning(
                    "Engine lever arm differs from lever_arm_config: %s",
                    IMU_LEVER_ARM_M,
                )
        return _engine


def with_engine(fn: Callable[[], T]) -> T:
    with _engine_lock:
        return fn()
