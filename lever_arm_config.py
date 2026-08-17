"""Fixed IMU lever arm (rotation center -> IMU), body frame meters.

Body frame: +X right, +Y forward, +Z up (looking along the bar from the front).

The BNO085 linear-acceleration report is Hillcrest fusion output (gravity removed).
During rotation it does not necessarily match the rigid-body model
  a_imu = omega_dot x r + omega x (omega x r)
because the game-rotation vector can lag (AR/VR stabilization) and the chip
fuses accel/gyro internally.  Per-axis CENTRIPETAL_GAIN_XYZ can absorb a
constant scale mismatch; keep lever-arm geometry at the measured chip offset.
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

# Per-axis scale on predicted centripetal/tangential term (1.0 = rigid body).
CENTRIPETAL_GAIN_XYZ = (1.0, 1.0, 1.0)

# Skip compensation when spin rate is below this [rad/s].
MIN_OMEGA_FOR_COMPENSATION_RAD_S = 0.5

# Ignore omega_dot when |alpha| exceeds this [rad/s^2] (noisy quat-derived gyro).
OMEGA_DOT_MAX_RAD_S2 = 12.0
