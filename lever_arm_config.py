"""Fixed sensor lever arms from the bar rotation center, collar body frame [m].

Body frame (fusion / viewer Y-up):
  +X right, +Y up, +Z forward along the bar.

Origin: center of rotation on the barbell (pivot).

Measured offsets from pivot:
  IMU (BNO085):    21.1 mm left, 7.42 mm below
  XM125 radar:     19.6 mm left, 32 mm below
  PMW3901 flow:    20.5 mm right, 34 mm below, pitched 14.5° about +X toward the back
"""

from __future__ import annotations

import math

# Pivot -> IMU (gyro reference)
IMU_LEVER_ARM_X_M = -0.0211
IMU_LEVER_ARM_Y_M = -0.00742
IMU_LEVER_ARM_Z_M = 0.0

IMU_LEVER_ARM_M = {
    "x": IMU_LEVER_ARM_X_M,
    "y": IMU_LEVER_ARM_Y_M,
    "z": IMU_LEVER_ARM_Z_M,
}

# Pivot -> downward radar (XM125)
RADAR_LEVER_ARM_X_M = -0.0196
RADAR_LEVER_ARM_Y_M = -0.032
RADAR_LEVER_ARM_Z_M = 0.0

RADAR_LEVER_ARM_M = {
    "x": RADAR_LEVER_ARM_X_M,
    "y": RADAR_LEVER_ARM_Y_M,
    "z": RADAR_LEVER_ARM_Z_M,
}

# Pivot -> optical flow sensor housing
FLOW_AT_COR_X_M = 0.0205
FLOW_AT_COR_Y_M = -0.034
FLOW_AT_COR_Z_M = 0.0

# IMU -> PMW3901 (used by mm_flow omega x r at flow chip)
FLOW_LEVER_ARM_X_M = FLOW_AT_COR_X_M - IMU_LEVER_ARM_X_M
FLOW_LEVER_ARM_Y_M = FLOW_AT_COR_Y_M - IMU_LEVER_ARM_Y_M
FLOW_LEVER_ARM_Z_M = FLOW_AT_COR_Z_M - IMU_LEVER_ARM_Z_M

FLOW_LEVER_ARM_M = {
    "x": FLOW_LEVER_ARM_X_M,
    "y": FLOW_LEVER_ARM_Y_M,
    "z": FLOW_LEVER_ARM_Z_M,
}

# Housing tilt about body +X (right): nose toward the back (+Z forward frame).
FLOW_MOUNT_PITCH_X_DEG = -14.5
FLOW_MOUNT_PITCH_X_RAD = math.radians(FLOW_MOUNT_PITCH_X_DEG)

# Per-axis scale on predicted centripetal/tangential term (1.0 = rigid body).
CENTRIPETAL_GAIN_XYZ = (1.0, 1.0, 1.0)

MIN_OMEGA_FOR_COMPENSATION_RAD_S = 0.5
OMEGA_DOT_MAX_RAD_S2 = 12.0
