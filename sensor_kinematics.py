"""Gravity-vector linear acceleration for telemetry (fusion Y-up world frame)."""

from __future__ import annotations

from fusion_settings import get_float_setting


def _world_gravity() -> tuple[float, float, float]:
    return (
        get_float_setting("WORLD_GRAVITY_X", 0.0),
        get_float_setting("WORLD_GRAVITY_Y", -9.81),
        get_float_setting("WORLD_GRAVITY_Z", 0.0),
    )


def _bno_gravity_for_quat() -> tuple[float, float, float]:
    """
    World gravity implied by BNO085 game-rotation quaternions.

    Game rotation is consistent with +X gravity in its native world frame, which
    differs from fusion EKF/viewer Y-up. Use this only for gravity subtraction
    before remapping linear accel into fusion world coordinates.
    """
    return (9.81, 0.0, 0.0)


def _rotate_vec(
    qx: float, qy: float, qz: float, qw: float,
    vx: float, vy: float, vz: float,
) -> tuple[float, float, float]:
    """Rotate a vector by quaternion (x, y, z, w). Matches tools/compensate_gravity_spin."""
    ix = qw * vx + qy * vz - qz * vy
    iy = qw * vy + qz * vx - qx * vz
    iz = qw * vz + qx * vy - qy * vx
    iw = -qx * vx - qy * vy - qz * vz
    return (
        ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy),
        iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz),
        iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx),
    )


def imu_vec_to_body(
    vec: dict[str, float] | tuple[float, float, float],
    imu_to_body: dict[str, float],
) -> tuple[float, float, float]:
    """Rotate IMU-frame vector into collar body frame (matches fusion.c fusion_imu_to_body)."""
    if isinstance(vec, dict):
        vx, vy, vz = float(vec["x"]), float(vec["y"]), float(vec["z"])
    else:
        vx, vy, vz = vec
    return _rotate_vec(
        float(imu_to_body["x"]),
        float(imu_to_body["y"]),
        float(imu_to_body["z"]),
        float(imu_to_body["w"]),
        vx, vy, vz,
    )


def gravity_body_from_quat(
    quat: dict[str, float],
    *,
    world_gravity: tuple[float, float, float] | None = None,
) -> tuple[float, float, float]:
    """Expected gravity in body frame from body-to-world quat and world gravity."""
    wg = world_gravity if world_gravity is not None else _world_gravity()
    return _rotate_vec(
        float(quat["x"]), float(quat["y"]), float(quat["z"]), float(quat["w"]),
        *wg,
    )


def body_to_world_vec(quat: dict[str, float], body: tuple[float, float, float]) -> tuple[float, float, float]:
    """Body-frame vector -> world frame for the quat's native world definition."""
    qx = float(quat["x"])
    qy = float(quat["y"])
    qz = float(quat["z"])
    qw = float(quat["w"])
    return _rotate_vec(-qx, -qy, -qz, qw, *body)


def bno_world_to_fusion_y_up(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Map BNO native world (+X gravity) into fusion/viewer Y-up world."""
    return (y, -x, z)


def world_linear_from_gravity_vector(
    quat_body: dict[str, float],
    gravity_imu: dict[str, float],
    *,
    imu_to_body: dict[str, float] | None = None,
) -> dict[str, float]:
    """
    BNO gravity vector minus gravity implied by body-to-world quat, in fusion Y-up world.

    quat_body: collar body attitude (imu quat composed with imu_to_body mount).
    gravity_imu: raw BNO gravity vector in IMU/sensor frame.
    """
    g_meas = (
        imu_vec_to_body(gravity_imu, imu_to_body)
        if imu_to_body is not None
        else (
            float(gravity_imu["x"]),
            float(gravity_imu["y"]),
            float(gravity_imu["z"]),
        )
    )
    g_exp = gravity_body_from_quat(quat_body, world_gravity=_bno_gravity_for_quat())
    lin_body = (
        g_meas[0] - g_exp[0],
        g_meas[1] - g_exp[1],
        g_meas[2] - g_exp[2],
    )
    bx, by, bz = body_to_world_vec(quat_body, lin_body)
    wx, wy, wz = bno_world_to_fusion_y_up(bx, by, bz)
    return {"x": wx, "y": wy, "z": wz}
