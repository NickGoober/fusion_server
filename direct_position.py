"""Direct collar position: radar height + orientation-rotated optical flow.

Bypasses the EKF flow/range updates. World frame is fusion Y-up:
  +X right, +Y up, +Z forward.
Origin is the first valid collar position; the floor is at y = -h0.
"""

from __future__ import annotations

import math
from typing import Any

from lever_arm_config import FLOW_MOUNT_PITCH_X_RAD, RADAR_LEVER_ARM_Y_M
from sensor_stream import imu_quat_to_body_frame

FLOW_SWAP_XY = True
FLOW_INVERT_X = True
FLOW_INVERT_Y = True
FLOW_FOV_DEG = 42.0
FLOW_NPIX = 35.0
# PMW3901 DELTA_X/Y registers are ~10× motion pixels (Crazyflie experimental).
# PX4 uses counts/385 rad; both give ~2.0–2.6 mm per count at 1 m height.
FLOW_RESOLUTION = 0.10
MIN_COUPLING = 0.35
MIN_HEIGHT_M = 0.02

# BNO game-rotation world (gravity along +X) → fusion/viewer Y-up.
# Vector map (x, y, z) → (y, −x, z) is −90 deg about +Z. Matches fusion.c.
_HS = math.sin(-0.25 * math.pi)
_HC = math.cos(-0.25 * math.pi)
_BNO_TO_FUSION = {"w": _HC, "x": 0.0, "y": 0.0, "z": _HS}


def _quat_mul(a: dict[str, float], b: dict[str, float]) -> dict[str, float]:
    return {
        "w": a["w"] * b["w"] - a["x"] * b["x"] - a["y"] * b["y"] - a["z"] * b["z"],
        "x": a["w"] * b["x"] + a["x"] * b["w"] + a["y"] * b["z"] - a["z"] * b["y"],
        "y": a["w"] * b["y"] - a["x"] * b["z"] + a["y"] * b["w"] + a["z"] * b["x"],
        "z": a["w"] * b["z"] + a["x"] * b["y"] - a["y"] * b["x"] + a["z"] * b["w"],
    }


def _quat_normalize(q: dict[str, float]) -> dict[str, float]:
    n = math.sqrt(q["w"] ** 2 + q["x"] ** 2 + q["y"] ** 2 + q["z"] ** 2)
    if n < 1e-12:
        return {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
    return {k: q[k] / n for k in ("w", "x", "y", "z")}


def fusion_body_attitude(
    imu_q: dict[str, float],
    imu_to_body: dict[str, float],
) -> dict[str, float]:
    """Collar body → fusion world, matching fusion_measured_body_attitude()."""
    q_ws = imu_quat_to_body_frame(imu_q, imu_to_body)
    return _quat_normalize(_quat_mul(_BNO_TO_FUSION, q_ws))


def _quat_to_R(q: dict[str, float]) -> list[list[float]]:
    q = _quat_normalize(q)
    qw, qx, qy, qz = q["w"], q["x"], q["y"], q["z"]
    return [
        [
            qw * qw + qx * qx - qy * qy - qz * qz,
            2 * qx * qy - 2 * qw * qz,
            2 * qx * qz + 2 * qw * qy,
        ],
        [
            2 * qx * qy + 2 * qw * qz,
            qw * qw - qx * qx + qy * qy - qz * qz,
            2 * qy * qz - 2 * qw * qx,
        ],
        [
            2 * qx * qz - 2 * qw * qy,
            2 * qy * qz + 2 * qw * qx,
            qw * qw - qx * qx - qy * qy + qz * qz,
        ],
    ]


def _map_flow_body(dx: int, dy: int) -> tuple[float, float]:
    raw_x, raw_y = float(dx), float(dy)
    bx = raw_y if FLOW_SWAP_XY else raw_x
    by = raw_x if FLOW_SWAP_XY else raw_y
    if FLOW_INVERT_X:
        bx = -bx
    if FLOW_INVERT_Y:
        by = -by
    bz = by * math.cos(FLOW_MOUNT_PITCH_X_RAD)
    return bx, bz


def meters_per_pixel(height_m: float, fov_deg: float = FLOW_FOV_DEG, npix: float = FLOW_NPIX) -> float:
    """Ground metres per PMW3901 register count at the given height.

    Crazyflie mm_flow.c:
      thetapix = 2*sin(FOV/2)   # 42° aperture → 0.71674 rad of ground width at z=1
      disp = count * 0.10 * z * thetapix / Npix
    Equivalent small-angle form used by flow_viewer: z * tan(FOV/Npix) * 0.10
    """
    npix = max(npix, 1.0)
    thetapix = 2.0 * math.sin(math.radians(fov_deg) * 0.5)
    return max(height_m, MIN_HEIGHT_M) * FLOW_RESOLUTION * thetapix / npix


def height_from_range_m(range_m: float, q_body: dict[str, float] | None) -> float:
    """Vertical distance from pivot to floor (world +Y)."""
    if range_m < MIN_HEIGHT_M:
        return MIN_HEIGHT_M
    if q_body is None:
        return range_m - RADAR_LEVER_ARM_Y_M
    R = _quat_to_R(q_body)
    # Body −Y (radar look) in world; coupling = |beam · world_up|.
    coupling = abs(-R[1][1])
    lever = -(R[1][1] * RADAR_LEVER_ARM_Y_M)
    if coupling < MIN_COUPLING:
        # Attitude too uncertain to tilt-correct — treat range as nadir height.
        return max(MIN_HEIGHT_M, range_m - RADAR_LEVER_ARM_Y_M)
    return max(MIN_HEIGHT_M, range_m * coupling + lever)


class DirectPositionTracker:
    """Integrate radar height and flow X/Z independently of the EKF."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.x_m = 0.0
        self.z_m = 0.0
        self.height_m = MIN_HEIGHT_M
        self.origin_height_m: float | None = None
        self._last_q: dict[str, float] | None = None

    @property
    def floor_offset_m(self) -> float:
        """World +Y distance from origin (first collar pose) down to the floor."""
        if self.origin_height_m is None:
            return 0.0
        return self.origin_height_m

    def position(self) -> dict[str, float]:
        y = 0.0 if self.origin_height_m is None else (self.height_m - self.origin_height_m)
        return {"x": self.x_m, "y": y, "z": self.z_m}

    def update(
        self,
        *,
        range_mm: int | None,
        flow: dict[str, Any] | None,
        imu_quat: dict[str, float] | None,
        imu_to_body: dict[str, float],
        fov_deg: float = FLOW_FOV_DEG,
        npix: float = FLOW_NPIX,
    ) -> None:
        if imu_quat is not None:
            self._last_q = fusion_body_attitude(imu_quat, imu_to_body)

        if range_mm is not None and range_mm > 0:
            self.height_m = height_from_range_m(range_mm / 1000.0, self._last_q)
            if self.origin_height_m is None:
                self.origin_height_m = self.height_m

        if not flow:
            return
        dx = int(flow.get("dx", 0))
        dy = int(flow.get("dy", 0))
        if dx == 0 and dy == 0:
            return
        quality = int(flow.get("quality", 255))
        if quality < 25:
            return

        bx, bz = _map_flow_body(dx, dy)
        mpp = meters_per_pixel(self.height_m, fov_deg, npix)
        dbx = bx * mpp
        dbz = bz * mpp

        if self._last_q is None:
            self.x_m += dbx
            self.z_m += dbz
            return

        # Yaw-only: keep full lateral magnitude, rotate body XZ into world XZ.
        R = _quat_to_R(self._last_q)
        yaw = math.atan2(R[0][2], R[2][2])
        c = math.cos(yaw)
        s = math.sin(yaw)
        self.x_m += c * dbx + s * dbz
        self.z_m += -s * dbx + c * dbz
