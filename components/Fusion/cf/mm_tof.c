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

  /* Legacy Z-only Jacobian — stable; height uses gravity axis when configured. */
  h[KC_STATE_Z] = 1.0f / r22;

  kalmanCoreScalarUpdate(this, &H, measuredDistance - predictedDistance, tof->stdDev);
}
