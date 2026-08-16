"""Fixed IMU lever arm (rotation center -> IMU), body frame meters.

Body frame: +X right, +Y forward, +Z up (looking along the bar from the front).
"""

# 21.1 mm left of rotation center
IMU_LEVER_ARM_X_M = -0.0211
# 7.42 mm below rotation center (user frame: displacement along Y)
IMU_LEVER_ARM_Y_M = -0.00742
IMU_LEVER_ARM_Z_M = 0.0

IMU_LEVER_ARM_M = {
    "x": IMU_LEVER_ARM_X_M,
    "y": IMU_LEVER_ARM_Y_M,
    "z": IMU_LEVER_ARM_Z_M,
}
