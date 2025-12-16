/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 *
 * kalman_supervisor.c - Supervises the kalman estimator and makes sure it
 * stays within bounds.
 */

/**
 * @file kalman_supervisor.c
 * @brief Safety net that detects runaway EKF states and requests resets.
 *
 * - Pipeline role: called from @ref kalmanTask() after every measurement cycle.
 *   If any position or velocity component exceeds a configurable bound, the task
 *   sets @c resetEstimation and the core state is re-initialized.
 * - Key structs: reuses @ref kalmanCoreData_t to inspect position/velocity,
 *   exposes @c maxPosition and @c maxVelocity as parameters under the `kalman`
 *   group so operators can tighten/loosen guard rails.
 * - Frames/units: positions are interpreted in the world frame [m], velocities
 *   are body-frame [m/s].
 * - Notes: bounds can be disabled by setting them to 0.0f (no supervision).
 */

#include "kalman_supervisor.h"

#include "param.h"

// The bounds on states, these shouldn't be hit...
float maxPosition = 100; //meters
float maxVelocity = 10; //meters per second

/**
 * @brief Check whether the EKF position/velocity states are inside configured bounds.
 *
 * @param this Kalman core data.
 * @return false if any XYZ position exceeds +/-maxPos or any body velocity exceeds +/-maxVel.
 */
bool kalmanSupervisorIsStateWithinBounds(const kalmanCoreData_t* this) {
  for (int i = 0; i < 3; i++) {
    if (maxPosition > 0.0f) {
      if (this->S[KC_STATE_X + i] > maxPosition) {
        return false;
      } else if (this->S[KC_STATE_X + i] < -maxPosition) {
        return false;
      }
    }

    if (maxVelocity > 0.0f) {
      if (this->S[KC_STATE_PX + i] > maxVelocity) {
        return false;
      } else if (this->S[KC_STATE_PX + i] < -maxVelocity) {
        return false;
      }
    }
  }

  return true;
}

PARAM_GROUP_START(kalman)
/**
 * @brief Maximum accepted coordinate before kalman supervisor
 * resets estimator
 */
  PARAM_ADD_CORE(PARAM_FLOAT, maxPos, &maxPosition)
  /**
 * @brief Maximum accepted velocity before kalman supervisor
 * resets estimator
 */
  PARAM_ADD_CORE(PARAM_FLOAT, maxVel, &maxVelocity)
PARAM_GROUP_STOP(kalman)
