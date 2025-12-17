/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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
 * estimator.h - State estimator interface
 *
 * This header is the hub for every estimator implementation (complementary, EKF,
 * UKF, out-of-tree). It 
 *  - declares the `stateEstimator*` API the stabilizer() loop calls to initialize, switch and run the estimator 
 *    - stateEstimatorInit(), stateEstimatorSwitchTo(), stateEstimator()
 *  - defines the `measurement_t` measurement struct and enqueue/dequeue functions
 *    - estimatorEnqueue(), estimatorDequeue()
 * Used to shuttle sensor data from drivers/decks into the active estimator.
 * Files under `src/modules/src/estimator/` consume this contract to implement
 * the runtime glue (queueing, switching, task ownership) for each algorithm.
 */
#pragma once

#include "autoconf.h"
#include "stabilizer_types.h"

typedef enum {
  StateEstimatorTypeAutoSelect = 0,
  StateEstimatorTypeComplementary,
#ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
  StateEstimatorTypeKalman,
#endif
#ifdef CONFIG_ESTIMATOR_UKF_ENABLE
  StateEstimatorTypeUkf,
#endif
#ifdef CONFIG_ESTIMATOR_OOT
  StateEstimatorTypeOutOfTree,
#endif
  StateEstimatorType_COUNT,
} StateEstimatorType;

typedef enum {
  MeasurementTypeTDOA,
  MeasurementTypePosition,
  MeasurementTypePose,
  MeasurementTypeDistance,
  MeasurementTypeTOF,
  MeasurementTypeAbsoluteHeight,
  MeasurementTypeFlow,
  MeasurementTypeYawError,
  MeasurementTypeSweepAngle,
  MeasurementTypeGyroscope,
  MeasurementTypeAcceleration,
  MeasurementTypeBarometer,
} MeasurementType;

typedef struct
{
  /** What kind of measurement is contained in `data`. */
  MeasurementType type;

  /**
   * Estimator measurement payload.
   *
   * NOTE:
   * - This union is NOT a list of onboard sensors.
   * - It is the common "measurement bus" used to feed the estimator from:
   *     (a) external positioning systems (Lighthouse, MoCap, UWB/LPS),
   *     (b) onboard sensors (IMU, barometer),
   *     (c) deck sensors (optical flow, range/ToF),
   *     (d) derived/correction terms produced by lighthouse processing.
   *
   * Some entries are raw-ish (e.g., sweep angles), some are processed pose/position,
   * and some are correction signals (e.g., yaw error).
   */
  union
{
  /* ---------- UWB / LPS constraints ---------- */

  /** UWB TDoA packet: time-difference-of-arrival constraint (hyperbolic). */
  tdoaMeasurement_t tdoa;

  /** UWB TWR: single-anchor range constraint (spherical). */
  distanceMeasurement_t distance;


  /* ---------- Lighthouse constraints ---------- */

  /** Lighthouse sweep angles (azimuth/elevation) from a base station. */
  sweepAngleMeasurement_t sweepAngle;

  /** Lighthouse-derived yaw correction term. */
  yawErrorMeasurement_t yawError;


  /* ---------- External absolute pose providers ---------- */

  /** Absolute position from MoCap or solved LPS pipeline. */
  positionMeasurement_t position;

  /** Absolute pose (position + quaternion) from MoCap or Lighthouse solver. */
  poseMeasurement_t pose;


  /* ---------- Deck-based exteroceptive sensors ---------- */

  /** Downward time-of-flight range (used for altitude / flow scaling). */
  tofMeasurement_t tof;

  /** General VL53-based height/reference measurement (mounting dependent). */
  heightMeasurement_t height;

  /** Optical flow pixel displacement measurement. */
  flowMeasurement_t flow;


  /* ---------- Onboard proprioceptive sensors ---------- */

  /** IMU gyroscope (body angular rates). */
  gyroscopeMeasurement_t gyroscope;

  /** IMU accelerometer (specific force). */
  accelerationMeasurement_t acceleration;

  /** Barometer-based altitude/pressure measurement. */
  barometerMeasurement_t barometer;
} data;
} measurement_t;

/**
 * @brief Create the measurement queue and start the requested estimator backend.
 *
 * @param estimator Preferred estimator (auto-select resolves to the default build option).
 */
void stateEstimatorInit(StateEstimatorType estimator);

/**
 * @brief Run the currently active estimator's self-test hook.
 */
bool stateEstimatorTest(void);

/**
 * @brief Switch to a different estimator backend at runtime.
 *
 * @param estimator Desired estimator (or auto-select).
 */
void stateEstimatorSwitchTo(StateEstimatorType estimator);

/**
 * @brief Execute one estimator update step from the stabilizer loop.
 *
 * Copies the latest state estimate into @p state using whichever backend is active.
 *
 * @param state Output pointer populated with position/attitude/velocity.
 * @param stabilizerStep Control-loop phase passed through from the caller.
 */
void stateEstimator(state_t *state, const stabilizerStep_t stabilizerStep);

/**
 * @brief Get the currently active estimator type.
 */
StateEstimatorType stateEstimatorGetType(void);

/**
 * @brief Human-readable name for the active estimator backend.
 */
const char* stateEstimatorGetName();

/**
 * @brief Enqueue a measurement_t packet for the active estimator to consume.
 *
 * Thread-safe helper used by deck drivers and sensor tasks.
 *
 * @param measurement Packet describing the measurement type and payload.
 */
void estimatorEnqueue(const measurement_t *measurement);

// These helper functions simplify the caller code, but cause additional memory copies
static inline void estimatorEnqueueTDOA(const tdoaMeasurement_t *tdoa)
{
  measurement_t m;
  m.type = MeasurementTypeTDOA;
  m.data.tdoa = *tdoa;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueuePosition(const positionMeasurement_t *position)
{
  measurement_t m;
  m.type = MeasurementTypePosition;
  m.data.position = *position;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueuePose(const poseMeasurement_t *pose)
{
  measurement_t m;
  m.type = MeasurementTypePose;
  m.data.pose = *pose;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueDistance(const distanceMeasurement_t *distance)
{
  measurement_t m;
  m.type = MeasurementTypeDistance;
  m.data.distance = *distance;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueTOF(const tofMeasurement_t *tof)
{
  measurement_t m;
  m.type = MeasurementTypeTOF;
  m.data.tof = *tof;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueAbsoluteHeight(const heightMeasurement_t *height)
{
  measurement_t m;
  m.type = MeasurementTypeAbsoluteHeight;
  m.data.height = *height;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueFlow(const flowMeasurement_t *flow)
{
  measurement_t m;
  m.type = MeasurementTypeFlow;
  m.data.flow = *flow;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueYawError(const yawErrorMeasurement_t *yawError)
{
  measurement_t m;
  m.type = MeasurementTypeYawError;
  m.data.yawError = *yawError;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueSweepAngles(const sweepAngleMeasurement_t *sweepAngle)
{
  measurement_t m;
  m.type = MeasurementTypeSweepAngle;
  m.data.sweepAngle = *sweepAngle;
  estimatorEnqueue(&m);
}

/**
 * @brief Non-blocking dequeue used by estimator implementations to drain the queue.
 *
 * @param measurement Output pointer that receives the dequeued entry.
 * @return true if a packet was removed from the queue.
 */
bool estimatorDequeue(measurement_t *measurement);

#ifdef CONFIG_ESTIMATOR_OOT
void estimatorOutOfTreeInit(void);
bool estimatorOutOfTreeTest(void);
void estimatorOutOfTree(state_t *state, const stabilizerStep_t stabilizerStep);
#endif
