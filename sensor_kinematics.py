"""Gravity-vector linear acceleration for telemetry (world frame)."""

from __future__ import annotations

from fusion_settings import get_float_setting


def _world_gravity() -> tuple[float, float, float]:
    return (
        get_float_setting("WORLD_GRAVITY_X", 0.0),
        get_float_setting("WORLD_GRAVITY_Y", -9.81),
        get_float_setting("WORLD_GRAVITY_Z", 0.0),
    )


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


def gravity_body_from_quat(quat: dict[str, float]) -> tuple[float, float, float]:
    """Expected gravity in body frame from game-rotation quat and configured world gravity."""
    return _rotate_vec(
        float(quat["x"]), float(quat["y"]), float(quat["z"]), float(quat["w"]),
        *_world_gravity(),
    )


def body_to_world_vec(quat: dict[str, float], body: tuple[float, float, float]) -> tuple[float, float, float]:
    """Body-frame vector -> fusion world frame (inverse of gravity_body_from_quat)."""
    qx = float(quat["x"])
    qy = float(quat["y"])
    qz = float(quat["z"])
    qw = float(quat["w"])
    return _rotate_vec(-qx, -qy, -qz, qw, *body)


def world_linear_from_gravity_vector(
    quat: dict[str, float],
    gravity_body: dict[str, float],
) -> dict[str, float]:
    """
    BNO wire type 1 gravity vector minus gravity implied by game-rotation quat,
    rotated into fusion world frame.
    """
    g_meas = (
        float(gravity_body["x"]),
        float(gravity_body["y"]),
        float(gravity_body["z"]),
    )
    g_exp = gravity_body_from_quat(quat)
    lin_body = (
        g_meas[0] - g_exp[0],
        g_meas[1] - g_exp[1],
        g_meas[2] - g_exp[2],
    )
    wx, wy, wz = body_to_world_vec(quat, lin_body)
    return {"x": wx, "y": wy, "z": wz}
