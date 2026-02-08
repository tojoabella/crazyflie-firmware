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
 * @file kalman_core.h
 * @brief Public interface for the Crazyflie Extended Kalman Filter core.
 *
 * This header describes how the FreeRTOS facing estimator task interacts with the math-heavy
 * implementation in @c src/modules/src/kalman_core. It exposes the state vector layout,
 * covariance container and predict/update hooks that measurement-model source files call.
 * - Pipeline role: provides the opaque @ref kalmanCoreData_t that the estimator task updates,
 *   and declares all sensor specific update helpers. The stabilizer reads data through
 *   @ref kalmanCoreExternalizeState().
 * - Key structs: @ref kalmanCoreData_t (state vector, quaternion, rotation matrix, covariance
 *   and timestamps) and @ref kalmanCoreParams_t (initial position/yaw, process and measurement
 *   noise settings).
 * - Key functions: initialization (@ref kalmanCoreInit(), @ref kalmanCoreDefaultParams()),
 *   predict and process noise updates (@ref kalmanCorePredict(), @ref kalmanCoreAddProcessNoise()),
 *   measurement entry points (@c kalmanCoreUpdateWith*) and post-processing helpers
 *   (@ref kalmanCoreFinalize(), @ref kalmanCoreExternalizeState()).
 * - Frames/units: positions live in the global/world frame [m], velocities are body-frame
 *   [m/s], attitude errors are stored as small-angle Rodrigues parameters [rad]. Gyro inputs
 *   are in rad/s and accelerometer inputs are in @c g.
 * - Notes on timing/numerics: timestamps use milliseconds (derived from FreeRTOS ticks),
 *   covariance symmetry is enforced after every update and Joseph form updates are used to
 *   minimize numerical drift.
 */

#include "cf_math.h"
#include "stabilizer_types.h"

// Indexes to access the quad's state, stored as a column vector
typedef enum
{
  KC_STATE_X, KC_STATE_Y, KC_STATE_Z, KC_STATE_PX, KC_STATE_PY, KC_STATE_PZ, KC_STATE_D0, KC_STATE_D1, KC_STATE_D2, KC_STATE_DIM
} kalmanCoreStateIdx_t;


// The data used by the kalman core implementation.
typedef struct {
  /**
   * Quadrocopter State
   *
   * The internally-estimated state is:
   * - X, Y, Z: the quad's position in the global frame
   * - PX, PY, PZ: the quad's velocity in its body frame
   * - D0, D1, D2: attitude error
   *
   * For more information, refer to the paper
   */
  float S[KC_STATE_DIM];

  // The quad's attitude as a quaternion (w,x,y,z)
  // We store as a quaternion to allow easy normalization (in comparison to a rotation matrix),
  // while also being robust against singularities (in comparison to euler angles)
  float q[4];

  // The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
  float R[3][3];

  // The covariance matrix
  __attribute__((aligned(4))) float P[KC_STATE_DIM][KC_STATE_DIM];
  arm_matrix_instance_f32 Pm;

  float baroReferenceHeight;

  // Quaternion used for initial orientation [w,x,y,z]
  float initialQuaternion[4];

  // Tracks whether an update to the state has been made, and the state therefore requires finalization
  bool isUpdated;

  uint32_t lastPredictionMs;
  uint32_t lastProcessNoiseUpdateMs;
} kalmanCoreData_t;

// The parameters used by the filter
typedef struct {
  // Initial variances, uncertain of position, but know we're stationary and roughly flat
  float stdDevInitialPosition_xy;
  float stdDevInitialPosition_z;
  float stdDevInitialVelocity;
  float stdDevInitialAttitude_rollpitch;
  float stdDevInitialAttitude_yaw;

  float procNoiseAcc_xy;
  float procNoiseAcc_z;
  float procNoiseVel;
  float procNoisePos;
  float procNoiseAtt;
  float measNoiseBaro;           // meters
  float measNoiseGyro_rollpitch; // radians per second
  float measNoiseGyro_yaw;       // radians per second

  float initialX;
  float initialY;
  float initialZ;

  // Initial yaw of the Crazyflie in radians.
  // 0 --- facing positive X
  // PI / 2 --- facing positive Y
  // PI --- facing negative X
  // 3 * PI / 2 --- facing negative Y
  float initialYaw;

  float attitudeReversion;
} kalmanCoreParams_t;

/**
 * @brief Initialize Kalman core parameters with default values
 *
 * This function exists primarily for Python bindings to initialize their own
 * kalmanCoreParams_t structs. In the firmware, default parameters are initialized
 * via a static initializer in estimator_kalman.c to avoid overwriting persistent
 * parameters loaded from storage.
 *
 * Default values are defined in kalman_core_params_defaults.h to maintain a
 * single source of truth.
 *
 * @param params Pointer to the parameter struct to initialize
 */
void kalmanCoreDefaultParams(kalmanCoreParams_t *params);

/**
 * @brief Initialize the Kalman core state vector, covariance and timestamps.
 *
 * Zeros the @ref kalmanCoreData_t structure, seeds y and yaw from @p params,
 * sets the initial quaternion/rotation matrix and initializes the covariance diagonal
 * to the configured variances. Called from @ref estimatorKalmanInit() (estimator_kalman.[h,c])
 * on startup and every time the supervisor requests a reset.
 *
 * @param this Kalman core data container to initialize.
 * @param params User configurable start state and noise parameters.
 * @param nowMs Current timestamp in milliseconds.
 */
void kalmanCoreInit(kalmanCoreData_t *this, const kalmanCoreParams_t *params, const uint32_t nowMs);

/**
 * @brief Perform a scalar barometer measurement update on the global Z position.
 *
 * Maintains a slowly varying baro reference while on the ground and feeds the
 * innovation into @ref kalmanCoreScalarUpdate(). Called whenever the estimator task
 * dequeues a @ref MeasurementTypeBarometer sample.
 *
 * @param this Kalman core data.
 * @param params Filter parameters (provides @c measNoiseBaro).
 * @param baroAsl Absolute altitude reported by the barometer [m].
 * @param quadIsFlying True when thrust is active, to freeze the reference height.
 */
void kalmanCoreUpdateWithBaro(kalmanCoreData_t *this, const kalmanCoreParams_t *params, float baroAsl, bool quadIsFlying);

/**
 * @brief Predict state and covariance forward using the IMU process model.
 *
 * Consumes subsampled acceleration/gyroscope data, discretizes the dynamics,
 * updates the covariance through @c A P A' and integrates the attitude quaternion.
 * Called at the EKF predict rate (~100 Hz) right before measurement updates.
 *
 * @param this Kalman core data.
 * @param params Process noise/attitude reversion gains.
 * @param acc Body-frame acceleration sample [g].
 * @param gyro Body-frame angular rate sample [rad/s].
 * @param nowMs Timestamp the sample represents [ms].
 * @param quadIsFlying True if thrust is active (switches acceleration handling).
 */
void kalmanCorePredict(kalmanCoreData_t *this, const kalmanCoreParams_t *params, Axis3f *acc, Axis3f *gyro, const uint32_t nowMs, bool quadIsFlying);

/**
 * @brief Add discretized process noise to the covariance between predict calls.
 *
 * Integrates the configured acceleration/velocity/attitude noise levels over the
 * elapsed time since @c lastProcessNoiseUpdateMs and clamps the covariance to ensure
 * symmetry and boundedness.
 *
 * @param this Kalman core data (provides @c P and timestamps).
 * @param params Process noise magnitudes.
 * @param nowMs Current timestamp [ms].
 */
void kalmanCoreAddProcessNoise(kalmanCoreData_t *this, const kalmanCoreParams_t *params, const uint32_t nowMs);

/**
 * @brief Finalization to incorporate attitude error into body attitude
 *
 * @param this Core data
 * @return true The state was finalized
 * @return false The state was not changed and did not require finalization
 */
bool kalmanCoreFinalize(kalmanCoreData_t* this);

/**
 * @brief Copy the internal EKF representation into the shared @ref state_t struct.
 *
 * Rotates body-frame velocities and accelerations to the global frame, converts yaw/pitch/roll
 * to degrees and stores the quaternion for downstream consumers. Called each estimator loop
 * right before the stabilizer task reads the state.
 *
 * @param this Kalman core data.
 * @param state Output pointer filled with world-frame estimates.
 * @param acc Latest body-frame acceleration sample used for external logging/control.
 */
void kalmanCoreExternalizeState(const kalmanCoreData_t* this, state_t *state, const Axis3f *acc);

/**
 * @brief Reset XY position/velocity states and decouple them from the rest of the EKF.
 *
 * Sets the affected covariance rows/columns to zero, inflates the diagonal and zeros
 * the state entries so that future measurements can safely pull the estimate back.
 * Used as a last-resort recovery when planar states diverge.
 *
 * @param this Kalman core data.
 */
void kalmanCoreDecoupleXY(kalmanCoreData_t* this);

/**
 * @brief Joseph-form scalar measurement update helper.
 *
 * Shared by all measurement models that map a single scalar sensor value to the
 * EKF state. Computes innovation covariance, Kalman gain, applies the state update
 * and enforces covariance symmetry/bounds.
 *
 * @param this Kalman core data.
 * @param Hm Measurement Jacobian row (1 x KC_STATE_DIM).
 * @param error Innovation value (measured - predicted).
 * @param stdMeasNoise Standard deviation of the measurement noise.
 */
void kalmanCoreScalarUpdate(kalmanCoreData_t* this, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise);

/**
 * @brief Kalman update variant that accepts pre-computed gain and weighted covariance.
 *
 * Used by the robust TDoA/TWR measurement models that run iterative re-weighting and
 * provide their own Kalman gain @p Km and weighted covariance @p P_w_m.
 *
 * @param this Kalman core data.
 * @param Hm Measurement Jacobian row.
 * @param Km Weighted Kalman gain vector.
 * @param P_w_m Weighted covariance to copy into @c this->P after the update.
 * @param error Innovation scalar (measurement residual).
 */
void kalmanCoreUpdateWithPKE(kalmanCoreData_t* this, arm_matrix_instance_f32 *Hm, arm_matrix_instance_f32 *Km, arm_matrix_instance_f32 *P_w_m, float error);
