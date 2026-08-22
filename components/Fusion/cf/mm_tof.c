#include "mm_tof.h"
#include "collar_gravity.h"

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  const float r22 = collarGravityBodyZCoupling((const float (*)[3])this->R, &this->worldGravity);
  if (r22 < 0.5f) {
    return;
  }

  const float pos[3] = {
    this->S[KC_STATE_X],
    this->S[KC_STATE_Y],
    this->S[KC_STATE_Z],
  };
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
