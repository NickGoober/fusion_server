#include "mm_flow.h"
#include "collar_gravity.h"
#include "log.h"
#include "platform_defaults.h"
#include "param.h"

#include <stddef.h>

#define FLOW_RESOLUTION 0.10f

static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;

static Axis3f flowdeckPos = { .axis = { FLOWDECK_POS_X, FLOWDECK_POS_Y, FLOWDECK_POS_Z } };
static Axis3f imuPivotPos = { .axis = { 0.0f, 0.0f, 0.0f } };

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
  float Npix = 35.0f;
  float thetapix = 0.71674f;

  float omegax_b = gyro->x * DEG_TO_RAD;
  float omegay_b = gyro->y * DEG_TO_RAD;
  float omegaz_b = gyro->z * DEG_TO_RAD;

  float dx_b = this->S[KC_STATE_PX];
  float dy_b = this->S[KC_STATE_PY];

  /* Height for flow scaling: gravity-aligned when configured, else legacy Z-up. */
  float z_g = 0.0f;
  const float gmag = sqrtf(
      this->worldGravity.wx * this->worldGravity.wx
      + this->worldGravity.wy * this->worldGravity.wy
      + this->worldGravity.wz * this->worldGravity.wz);
  if (gmag > 1e-3f) {
    const float pos[3] = {
      this->S[KC_STATE_X],
      this->S[KC_STATE_Y],
      this->S[KC_STATE_Z],
    };
    z_g = collarGravityHeightM(pos, &this->worldGravity);
    if (this->range_height_hint_m > z_g) {
      z_g = this->range_height_hint_m;
    }
  } else if (this->S[KC_STATE_Z] < 0.02f) {
    z_g = 0.02f;
  } else {
    z_g = this->S[KC_STATE_Z];
  }

  const float r22 = collarGravityBodyZCoupling((const float (*)[3])this->R, &this->worldGravity);

  float v_imu_bx_add = omegay_b * imuPivotPos.z - omegaz_b * imuPivotPos.y;
  float v_imu_by_add = omegaz_b * imuPivotPos.x - omegax_b * imuPivotPos.z;
  float v_flow_bx_add = omegay_b * flowdeckPos.z - omegaz_b * flowdeckPos.y;
  float v_flow_by_add = omegaz_b * flowdeckPos.x - omegax_b * flowdeckPos.z;

  float v_cam_bx = dx_b + v_imu_bx_add + v_flow_bx_add;
  float v_cam_by = dy_b + v_imu_by_add + v_flow_by_add;

  float hx[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
  predictedNX = (flow->dt * Npix / thetapix) * ((v_cam_bx * r22 / z_g) - omegay_b);
  measuredNX = flow->dpixelx * FLOW_RESOLUTION;

  /* Jacobian: only height (Z) and body velocity — do not pull X/Y world position. */
  hx[KC_STATE_Z] = (Npix * flow->dt / thetapix) * ((r22 * v_cam_bx) / (-z_g * z_g));
  hx[KC_STATE_PX] = (Npix * flow->dt / thetapix) * (r22 / z_g);

  kalmanCoreScalarUpdate(this, &Hx, (measuredNX - predictedNX), flow->stdDevX * FLOW_RESOLUTION);

  float hy[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};
  predictedNY = (flow->dt * Npix / thetapix) * ((v_cam_by * r22 / z_g) + omegax_b);
  measuredNY = flow->dpixely * FLOW_RESOLUTION;

  hy[KC_STATE_Z] = (Npix * flow->dt / thetapix) * ((r22 * v_cam_by) / (-z_g * z_g));
  hy[KC_STATE_PY] = (Npix * flow->dt / thetapix) * (r22 / z_g);

  kalmanCoreScalarUpdate(this, &Hy, (measuredNY - predictedNY), flow->stdDevY * FLOW_RESOLUTION);
}

LOG_GROUP_START(kalman_pred)
  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
LOG_GROUP_STOP(kalman_pred)

PARAM_GROUP_START(flowdeck)
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_x, &flowdeckPos.x)
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_y, &flowdeckPos.y)
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_z, &flowdeckPos.z)
PARAM_GROUP_STOP(flowdeck)
