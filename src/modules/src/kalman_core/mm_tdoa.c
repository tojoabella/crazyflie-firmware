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

#include "mm_tdoa.h"
#include "test_support.h"

#if CONFIG_ESTIMATOR_KALMAN_TDOA_OUTLIERFILTER_FALLBACK
#include "outlierFilterTdoaSteps.h"
#endif

/**
 * @file mm_tdoa.c
 * @brief TDoA measurement model and gating logic for the Kalman core.
 *
 * - Pipeline role: consumes TDoA packets from the Loco Positioning System and
 *   fuses them into world-frame positions. Includes two outlier filters controlled
 *   by @c CONFIG_ESTIMATOR_KALMAN_TDOA_OUTLIERFILTER_FALLBACK.
 * - Key structs: @ref tdoaMeasurement_t, @ref OutlierFilterTdoaState_t,
 *   @ref vector_t (Jacobian used for the legacy step filter).
 * - Frames/units: anchor/state positions are meters in the world frame.
 * - Notes: measurement Jacobian is the difference of normalized anchor direction
 *   vectors; gating rejects impossible geometry and large residuals.
 */

/**
 * @brief Fuse a UWB time-difference-of-arrival measurement.
 *
 * Computes the predicted distance difference from the current state, builds the
 * Jacobian for XYZ, runs the configured outlier filter to gate impossible residuals
 * and forwards the innovation to @ref kalmanCoreScalarUpdate().
 *
 * @param this Kalman core data.
 * @param tdoa Measurement packet.
 * @param nowMs Timestamp used by the integrator-based outlier filter.
 * @param outlierFilterState Integrator/legacy filter context shared across updates.
 */
void kalmanCoreUpdateWithTdoa(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa, const uint32_t nowMs, OutlierFilterTdoaState_t* outlierFilterState)
{
  /**
   * Measurement equation:
   * dR = dT + d1 - d0
   */

  float measurement = tdoa->distanceDiff;

  // predict based on current state
  float x = this->S[KC_STATE_X];
  float y = this->S[KC_STATE_Y];
  float z = this->S[KC_STATE_Z];

  float x1 = tdoa->anchorPositions[1].x, y1 = tdoa->anchorPositions[1].y, z1 = tdoa->anchorPositions[1].z;
  float x0 = tdoa->anchorPositions[0].x, y0 = tdoa->anchorPositions[0].y, z0 = tdoa->anchorPositions[0].z;

  float dx1 = x - x1;
  float dy1 = y - y1;
  float dz1 = z - z1;

  float dy0 = y - y0;
  float dx0 = x - x0;
  float dz0 = z - z0;

  float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
  float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));

  float predicted = d1 - d0;
  float error = measurement - predicted;

  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  if ((d0 != 0.0f) && (d1 != 0.0f)) {
    h[KC_STATE_X] = (dx1 / d1 - dx0 / d0);
    h[KC_STATE_Y] = (dy1 / d1 - dy0 / d0);
    h[KC_STATE_Z] = (dz1 / d1 - dz0 / d0);

  #if CONFIG_ESTIMATOR_KALMAN_TDOA_OUTLIERFILTER_FALLBACK
    vector_t jacobian = {
      .x = h[KC_STATE_X],
      .y = h[KC_STATE_Y],
      .z = h[KC_STATE_Z],
    };

    point_t estimatedPosition = {
      .x = this->S[KC_STATE_X],
      .y = this->S[KC_STATE_Y],
      .z = this->S[KC_STATE_Z],
    };

    bool sampleIsGood = outlierFilterTdoaValidateSteps(tdoa, error, &jacobian, &estimatedPosition);
    #else
    bool sampleIsGood = outlierFilterTdoaValidateIntegrator(outlierFilterState, tdoa, error, nowMs);
    #endif

    if (sampleIsGood) {
      kalmanCoreScalarUpdate(this, &H, error, tdoa->stdDev);
    }
  }
}
