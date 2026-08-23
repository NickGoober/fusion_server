/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include "kalman_core.h"

// Measurements of flow (dnx, dny)
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro);

// Flow sensor position relative to the IMU / body origin [m], body frame.
void mm_flow_set_position(float x_m, float y_m, float z_m);
void mm_flow_get_position(float *x_m, float *y_m, float *z_m);

// IMU offset from the device rotation center to the gyro/IMU [m], body frame.
void mm_flow_set_imu_pivot_offset(float x_m, float y_m, float z_m);
void mm_flow_get_imu_pivot_offset(float *x_m, float *y_m, float *z_m);

/** Extra pitch about body +X after Y-up flow-plane mapping (rad). */
void mm_flow_set_mount_pitch_rad(float pitch_x_rad);
float mm_flow_get_mount_pitch_rad(void);
