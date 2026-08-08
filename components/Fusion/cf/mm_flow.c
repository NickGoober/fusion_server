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
#include <stdio.h>
#include "mm_flow.h"
#include "log.h"
#include "platform_defaults.h"
#include "param.h"

#define FLOW_RESOLUTION 0.10f //We get the measurements in 10x the motion pixels (experimentally measured)

// TODO remove the temporary test variables (used for logging)
static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;

static Axis3f flowdeckPos = { .axis = { FLOWDECK_POS_X, FLOWDECK_POS_Y, FLOWDECK_POS_Z } }; // IMU -> flow
static Axis3f imuPivotPos = { .axis = { 0.0f, 0.0f, 0.0f } }; // rotation center -> IMU

void mm_flow_set_position(float x_m, float y_m, float z_m)
{
  flowdeckPos.x = x_m;
  flowdeckPos.y = y_m;
  flowdeckPos.z = z_m;
}

void mm_flow_get_position(float *x_m, float *y_m, float *z_m)
{
  if (x_m != NULL) {
    *x_m = flowdeckPos.x;
  }
  if (y_m != NULL) {
    *y_m = flowdeckPos.y;
  }
  if (z_m != NULL) {
    *z_m = flowdeckPos.z;
  }
}

void mm_flow_set_imu_pivot_offset(float x_m, float y_m, float z_m)
{
  imuPivotPos.x = x_m;
  imuPivotPos.y = y_m;
  imuPivotPos.z = z_m;
}

void mm_flow_get_imu_pivot_offset(float *x_m, float *y_m, float *z_m)
{
  if (x_m != NULL) {
    *x_m = imuPivotPos.x;
  }
  if (y_m != NULL) {
    *y_m = imuPivotPos.y;
  }
  if (z_m != NULL) {
    *z_m = imuPivotPos.z;
  }
}

void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro)
{
  // Inclusion of flow measurements in the EKF done by two scalar updates

  // ~~~ Camera constants ~~~
  // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
  float Npix = 35.0;         // [pixels] (same in x and y)
  float thetapix = 0.71674f; // [rad] 2*sin(42/2); 42 degrees is the angle of aperture, here we computed the corresponding ground length

  //~~~ Extract states ~~~
  // Body rates
  float omegax_b = gyro->x * DEG_TO_RAD;
  float omegay_b = gyro->y * DEG_TO_RAD;
  float omegaz_b = gyro->z * DEG_TO_RAD;

  // Velocities in body frame
  float dx_b = this->S[KC_STATE_PX];
  float dy_b = this->S[KC_STATE_PY];

// Height above origin
  float z_g = 0.0f;

  // Saturate height in prediction and correction to avoid singularities.
  // Lowered from 0.1m (drone) to 0.02m (barbell on the floor) to prevent 
  // velocity scaling distortion when the barbell is resting or rolling on the ground.
  if (this->S[KC_STATE_Z] < 0.02f) {
      z_g = 0.02f;
  } else {
      z_g = this->S[KC_STATE_Z];
  }

  // Lever-arm induced translational velocity at IMU (rotation center -> IMU)
  // and at the flow sensor (IMU -> flow).
  float v_imu_bx_add = omegay_b * imuPivotPos.z - omegaz_b * imuPivotPos.y;
  float v_imu_by_add = omegaz_b * imuPivotPos.x - omegax_b * imuPivotPos.z;
  float v_flow_bx_add = omegay_b * flowdeckPos.z - omegaz_b * flowdeckPos.y;
  float v_flow_by_add = omegaz_b * flowdeckPos.x - omegax_b * flowdeckPos.z;

  // Effective camera point velocities in body frame
  float v_cam_bx = dx_b + v_imu_bx_add + v_flow_bx_add;
  float v_cam_by = dy_b + v_imu_by_add + v_flow_by_add;

  // X velocity prediction and update
  // predicts the number of accumulated pixels in the x-direction
  float hx[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
  predictedNX = (flow->dt * Npix / thetapix) * ((v_cam_bx * this->R[2][2] / z_g) - omegay_b);
  measuredNX = flow->dpixelx*FLOW_RESOLUTION;

  // derive measurement equation with respect to dx (and z?)
  hx[KC_STATE_Z]  = (Npix * flow->dt / thetapix) * ((this->R[2][2] * v_cam_bx) / (-z_g * z_g));
  hx[KC_STATE_PX] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);



    // kalmanCoreScalarUpdate(this, &H, ...);
  //First update
  kalmanCoreScalarUpdate(this, &Hx, (measuredNX-predictedNX), flow->stdDevX*FLOW_RESOLUTION);

  // Y velocity prediction and update
  float hy[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};
  predictedNY = (flow->dt * Npix / thetapix ) * ((v_cam_by * this->R[2][2] / z_g) + omegax_b);
  measuredNY = flow->dpixely*FLOW_RESOLUTION;

  // derive measurement equation with respect to dy (and z?)
  hy[KC_STATE_Z]  = (Npix * flow->dt / thetapix) * ((this->R[2][2] * v_cam_by) / (-z_g * z_g));
  hy[KC_STATE_PY] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);

  // Second update
  kalmanCoreScalarUpdate(this, &Hy, (measuredNY-predictedNY), flow->stdDevY*FLOW_RESOLUTION);
}

/**
 * Predicted and measured values of the X and Y direction of the flowdeck
 */
LOG_GROUP_START(kalman_pred)

/**
 * @brief Flow sensor predicted dx  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowX?
 */
  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
/**
 * @brief Flow sensor predicted dy  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowY?
 */
  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
/**
 * @brief Flow sensor measured dx  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaX, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
/**
 * @brief Flow sensor measured dy  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaY, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
LOG_GROUP_STOP(kalman_pred)

/**
 * Flowdeck properties
 */
PARAM_GROUP_START(flowdeck)
  /**
   * @brief Flow deck position X (in meters, body frame)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_x, &flowdeckPos.x)
  /**
   * @brief Flow deck position Y (in meters, body frame)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_y, &flowdeckPos.y)
  /**
   * @brief Flow deck position Z (in meters, body frame)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_z, &flowdeckPos.z)
PARAM_GROUP_STOP(flowdeck)
