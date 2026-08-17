"""Remove rigid-body offset acceleration at the IMU (centripetal / tangential)."""

from __future__ import annotations

import math
from typing import Mapping

from lever_arm_config import (
    CENTRIPETAL_GAIN_XYZ,
    IMU_LEVER_ARM_M,
    MIN_OMEGA_FOR_COMPENSATION_RAD_S,
    OMEGA_DOT_MAX_RAD_S2,
)

Vec3 = tuple[float, float, float]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _vec3_from_mapping(v: Mapping[str, float]) -> Vec3:
    return (float(v["x"]), float(v["y"]), float(v["z"]))


def kinematic_accel(
    omega: Vec3,
    omega_dot: Vec3,
    arm: Vec3,
) -> Vec3:
    """a_offset = omega_dot x r + omega x (omega x r)  [m/s^2]."""
    w_cross_r = _cross(omega, arm)
    centripetal = _cross(omega, w_cross_r)
    tangential = _cross(omega_dot, arm)
    return (
        tangential[0] + centripetal[0],
        tangential[1] + centripetal[1],
        tangential[2] + centripetal[2],
    )


def compensate_linear_accel(
    accel: Mapping[str, float],
    gyro: Mapping[str, float],
    *,
    arm: Mapping[str, float] | None = None,
    prev_gyro: Mapping[str, float] | None = None,
    dt_s: float | None = None,
    gain_xyz: Vec3 | None = None,
) -> dict[str, float]:
    """
    Estimate linear acceleration at the rotation center (body frame).

    a_center = a_imu - gain * (omega_dot x r + omega x (omega x r))
    """
    a = _vec3_from_mapping(accel)
    omega = _vec3_from_mapping(gyro)
    r = _vec3_from_mapping(arm or IMU_LEVER_ARM_M)
    gains = gain_xyz or CENTRIPETAL_GAIN_XYZ

    omega_mag = math.sqrt(omega[0] ** 2 + omega[1] ** 2 + omega[2] ** 2)
    if omega_mag < MIN_OMEGA_FOR_COMPENSATION_RAD_S:
        return {"x": a[0], "y": a[1], "z": a[2]}

    if prev_gyro is not None and dt_s is not None and dt_s > 1e-6:
        p = _vec3_from_mapping(prev_gyro)
        omega_dot = (
            (omega[0] - p[0]) / dt_s,
            (omega[1] - p[1]) / dt_s,
            (omega[2] - p[2]) / dt_s,
        )
        alpha_mag = math.sqrt(
            omega_dot[0] ** 2 + omega_dot[1] ** 2 + omega_dot[2] ** 2,
        )
        if alpha_mag > OMEGA_DOT_MAX_RAD_S2:
            omega_dot = (0.0, 0.0, 0.0)
    else:
        omega_dot = (0.0, 0.0, 0.0)

    offset = kinematic_accel(omega, omega_dot, r)
    return {
        "x": a[0] - gains[0] * offset[0],
        "y": a[1] - gains[1] * offset[1],
        "z": a[2] - gains[2] * offset[2],
    }


def accel_magnitude(accel: Mapping[str, float]) -> float:
    return math.sqrt(accel["x"] ** 2 + accel["y"] ** 2 + accel["z"] ** 2)
