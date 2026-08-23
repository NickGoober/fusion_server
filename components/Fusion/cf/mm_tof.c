#include "mm_tof.h"

#include "collar_gravity.h"

typedef struct {
  float x;
  float y;
  float z;
} mm_tof_vec3_t;

static mm_tof_vec3_t s_rangeLeverArm = {0.0f, 0.0f, 0.0f};

void mm_tof_set_lever_arm(float x_m, float y_m, float z_m)
{
  s_rangeLeverArm.x = x_m;
  s_rangeLeverArm.y = y_m;
  s_rangeLeverArm.z = z_m;
}

void mm_tof_get_lever_arm(float *x_m, float *y_m, float *z_m)
{
  if (x_m != NULL) {
    *x_m = s_rangeLeverArm.x;
  }
  if (y_m != NULL) {
    *y_m = s_rangeLeverArm.y;
  }
  if (z_m != NULL) {
    *z_m = s_rangeLeverArm.z;
  }
}

static void mm_tof_radar_position_world(
    const kalmanCoreData_t *this,
    float pos_out[3])
{
  const float rx = s_rangeLeverArm.x;
  const float ry = s_rangeLeverArm.y;
  const float rz = s_rangeLeverArm.z;
  pos_out[0] = this->S[KC_STATE_X]
      + this->R[0][0] * rx + this->R[0][1] * ry + this->R[0][2] * rz;
  pos_out[1] = this->S[KC_STATE_Y]
      + this->R[1][0] * rx + this->R[1][1] * ry + this->R[1][2] * rz;
  pos_out[2] = this->S[KC_STATE_Z]
      + this->R[2][0] * rx + this->R[2][1] * ry + this->R[2][2] * rz;
}

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  const float r22 = collarGravityBodyZCoupling((const float (*)[3])this->R, &this->worldGravity);
  if (r22 < 0.5f) {
    return;
  }

  float pos[3];
  mm_tof_radar_position_world(this, pos);

  const float gmag = sqrtf(
      this->worldGravity.wx * this->worldGravity.wx
      + this->worldGravity.wy * this->worldGravity.wy
      + this->worldGravity.wz * this->worldGravity.wz);
  const float height = (gmag > 1e-3f)
      ? collarGravityHeightM(pos, &this->worldGravity)
      : this->S[KC_STATE_Z];

  const float predictedDistance = height / r22;
  const float measuredDistance = tof->distance;

  float jac[3];
  collarGravityHeightStateJacobian(&this->worldGravity, 1.0f / r22, jac);
  h[KC_STATE_X] = jac[0];
  h[KC_STATE_Y] = jac[1];
  h[KC_STATE_Z] = jac[2];

  kalmanCoreScalarUpdate(this, &H, measuredDistance - predictedDistance, tof->stdDev);
}
