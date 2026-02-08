/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * \verbatim
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D'Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D'Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 * \endverbatim
 *
 * ============================================================================
 */

#pragma once

/**
 * @file estimator_kalman.h
 * @brief Public entry points for the Crazyflie Kalman state estimator.
 *
 * This header is consumed by the stabilizer task and deck drivers that need to
 * query the EKF outside of the logging/parameter infrastructure.
 * - Pipeline role: exposes start/stop/test hooks for the estimator task,
 *   provides helper accessors for position/rotation and declares the main loop
 *   function called from the stabilizer.
 * - Key functions: @ref estimatorKalmanInit(), @ref estimatorKalman(),
 *   @ref estimatorKalmanTaskInit(), @ref estimatorKalmanGetEstimatedPos().
 * - Frames/units: positions returned in meters in the world frame, rotation
 *   matrices are 3x3 row-major view of the EKF @c R matrix.
 * - Notes: The RTOS task initialization is split between @ref estimatorKalmanTaskInit()
 *   (called at boot) and @ref estimatorKalmanInit() (called when the estimator is
 *   selected or reset).
 */

#include <stdint.h>
#include "stabilizer_types.h"

/**
 * @brief Initialize the Kalman estimator state and measurement filters.
 *
 * Called when the estimator is activated or when a reset is requested.
 * Used in estimatorFunctions[] in estimator.c.
 */
void estimatorKalmanInit(void);

/**
 * @brief Return true when the Kalman estimator task/structures have been created.
 */
bool estimatorKalmanTest(void);

/**
 * @brief Stabilizer facing entry point. Copies the current estimate and wakes the Kalman task.
 *
 * @param state Output pointer filled with the current @ref state_t.
 * @param stabilizerStep Provides the caller's control-loop phase (unused).
 */
void estimatorKalman(state_t *state, const stabilizerStep_t stabilizerStep);

/**
 * @brief Allocate semaphores/mutexes and create the FreeRTOS task that runs the EKF.
 */
void estimatorKalmanTaskInit();

/**
 * @brief Return true when @ref estimatorKalmanTaskInit() completed successfully.
 */
bool estimatorKalmanTaskTest();

/**
 * @brief Convenience accessor copying the latest world-frame position estimate.
 *
 * @param pos Pointer populated with XYZ in meters.
 */
void estimatorKalmanGetEstimatedPos(point_t* pos);

/**
 * @brief Copy the current EKF rotation matrix (row-major 3x3) into the provided buffer.
 *
 * @param rotationMatrix Pointer to an array of 9 floats written row-major.
 */
void estimatorKalmanGetEstimatedRot(float * rotationMatrix);
