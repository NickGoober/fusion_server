#include "mm_flow.h"
#include "collar_gravity.h"
#include "log.h"
#include "platform_defaults.h"
#include "param.h"

#include <math.h>
#include <stddef.h>

#define FLOW_RESOLUTION 0.10f

static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;

static Axis3f flowdeckPos = { .axis = { FLOWDECK_POS_X, FLOWDECK_POS_Y, FLOWDECK_POS_Z } };
static Axis3f imuPivotPos = { .axis = { 0.0f, 0.0f, 0.0f } };
static float flowMountPitchRad = 0.0f;

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

void mm_flow_set_mount_pitch_rad(float pitch_x_rad)
{
  flowMountPitchRad = pitch_x_rad;
}

float mm_flow_get_mount_pitch_rad(void)
{
  return flowMountPitchRad;
}

static void flow_velocity_in_sensor_plane(
    kalmanCoreData_t* this,
    const Axis3f *gyro,
    float *vs_x,
    float *vs_y)
{
  const float omegax_b = gyro->x * DEG_TO_RAD;
  const float omegay_b = gyro->y * DEG_TO_RAD;
  const float omegaz_b = gyro->z * DEG_TO_RAD;

  float vbx = this->S[KC_STATE_PX];
  float vby = this->S[KC_STATE_PY];
  float vbz = this->S[KC_STATE_PZ];

  const float v_imu_x = omegay_b * imuPivotPos.z - omegaz_b * imuPivotPos.y;
  const float v_imu_y = omegaz_b * imuPivotPos.x - omegax_b * imuPivotPos.z;
  const float v_imu_z = omegax_b * imuPivotPos.y - omegay_b * imuPivotPos.x;

  const float v_flow_x = omegay_b * flowdeckPos.z - omegaz_b * flowdeckPos.y;
  const float v_flow_y = omegaz_b * flowdeckPos.x - omegax_b * flowdeckPos.z;
  const float v_flow_z = omegax_b * flowdeckPos.y - omegay_b * flowdeckPos.x;

  vbx += v_imu_x + v_flow_x;
  vby += v_imu_y + v_flow_y;
  vbz += v_imu_z + v_flow_z;

  /* Y-up collar: flow looks down (-Y); sensor plane is body X (right) x Z (forward). */
  const float nom_x = vbx;
  const float nom_y = vbz;
  const float nom_z = -vby;

  const float cp = cosf(flowMountPitchRad);
  const float sp = sinf(flowMountPitchRad);
  *vs_x = nom_x;
  *vs_y = cp * nom_y - sp * nom_z;
}

void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro)
{
  float Npix = 35.0f;
  float thetapix = 0.71674f;

  float omegax_b = gyro->x * DEG_TO_RAD;
  float omegay_b = gyro->y * DEG_TO_RAD;
  float omegaz_b = gyro->z * DEG_TO_RAD;

  float v_cam_bx;
  float v_cam_by;
  flow_velocity_in_sensor_plane(this, gyro, &v_cam_bx, &v_cam_by);

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

  float hx[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
  predictedNX = (flow->dt * Npix / thetapix) * ((v_cam_bx * r22 / z_g) - omegay_b);
  measuredNX = flow->dpixelx * FLOW_RESOLUTION;

  float gh_x = 0.0f;
  float gh_y = -1.0f;
  float gh_z = 0.0f;
  if (gmag > 1e-3f) {
    collarGravityHat(&this->worldGravity, &gh_x, &gh_y, &gh_z);
  }
  {
    const float scale = (Npix * flow->dt / thetapix) * (r22 * v_cam_bx) / (z_g * z_g);
    hx[KC_STATE_X] = scale * gh_x;
    hx[KC_STATE_Y] = scale * gh_y;
    hx[KC_STATE_Z] = scale * gh_z;
  }
  hx[KC_STATE_PX] = (Npix * flow->dt / thetapix) * (r22 / z_g);

  kalmanCoreScalarUpdate(this, &Hx, (measuredNX - predictedNX), flow->stdDevX * FLOW_RESOLUTION);

  float hy[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};
  predictedNY = (flow->dt * Npix / thetapix) * ((v_cam_by * r22 / z_g) + omegax_b);
  measuredNY = flow->dpixely * FLOW_RESOLUTION;

  {
    const float scale = (Npix * flow->dt / thetapix) * (r22 * v_cam_by) / (z_g * z_g);
    hy[KC_STATE_X] = scale * gh_x;
    hy[KC_STATE_Y] = scale * gh_y;
    hy[KC_STATE_Z] = scale * gh_z;
  }
  hy[KC_STATE_PX] = 0.0f;
  {
    const float vel_scale = (Npix * flow->dt / thetapix) * (r22 / z_g);
    const float cp = cosf(flowMountPitchRad);
    const float sp = sinf(flowMountPitchRad);
    hy[KC_STATE_PZ] = vel_scale * cp;
    hy[KC_STATE_PY] = vel_scale * sp;
  }

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
