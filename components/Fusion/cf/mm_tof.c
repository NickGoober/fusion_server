#include "mm_tof.h"
#include "collar_gravity.h"

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  const float coupling = collarGravityBodyZCoupling((const float (*)[3])this->R, &this->worldGravity);
  if (coupling < 0.5f) {
    return;
  }

  const float pos[3] = {
    this->S[KC_STATE_X],
    this->S[KC_STATE_Y],
    this->S[KC_STATE_Z],
  };
  const float height = collarGravityHeightM(pos, &this->worldGravity);
  const float predictedDistance = height / coupling;
  const float measuredDistance = tof->distance; // [m]

  float gh_x = 0.0f;
  float gh_y = 0.0f;
  float gh_z = 1.0f;
  collarGravityHat(&this->worldGravity, &gh_x, &gh_y, &gh_z);

  h[KC_STATE_X] = -gh_x / coupling;
  h[KC_STATE_Y] = -gh_y / coupling;
  h[KC_STATE_Z] = -gh_z / coupling;

  kalmanCoreScalarUpdate(this, &H, measuredDistance - predictedDistance, tof->stdDev);
}
