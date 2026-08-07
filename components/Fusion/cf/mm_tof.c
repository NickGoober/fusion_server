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

#include "mm_tof.h"

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // R[2][2] is the cosine of the tilt angle. 
  // 0.5f allows up to a 60-degree tilt before rejecting the sample.
  if (this->R[2][2] > 0.5f) {
    float cos_theta = this->R[2][2];
    float predictedDistance = this->S[KC_STATE_Z] / cos_theta;
    float measuredDistance = tof->distance; // [m]

    // The Jacobian maps the vertical Z state to the slanted hypotenuse measurement
    h[KC_STATE_Z] = 1.0f / cos_theta; 

    // Scalar update
    kalmanCoreScalarUpdate(this, &H, measuredDistance - predictedDistance, tof->stdDev);
  }
}
