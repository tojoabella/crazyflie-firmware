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
 * BIBTEX ENTRIES:
 * \verbatim
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 * \endverbatim
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from OS related functionality
 * 2021.03.15, Wolfgang Hoenig: Refactored queue handling
 */

/**
 * @file estimator_kalman.c
 * @brief FreeRTOS task layer that orchestrates the Kalman core.
 *
 * Main EKF file. This file bridges the OS/sensor world with the pure Kalman math in
 * @ref kalman_core.c. It continually drains the measurement queue to update the estimated 
 * state (kalmanTask), then makes it available to the stabilizer loop.
 * 
 * - Pipeline: kalmanTask() FreeRTOS task continually loops, 
 *   updating the internal coreData state and then copying a simpler view to
 *   publishedEstimatorState, for consumption by the stabilizer loop (500 Hz) via estimatorKalman().
 *
 *
 * This module splits estimator work into two cooperating contexts:
 *
 * - `estimatorKalman()` runs in the **stabilizer/control loop**
 *   (high-rate, latency-sensitive). It:
 *     1) acquires the mutex guarding the estimator's simple snapshot of the state,
 *     2) copies the state
 *     3) releases the mutex,
 *     4) releases the kalmanTask mutex, signaling it to run another state estimation update
 *
 * - `kalmanTask()` runs in its own **FreeRTOS task context**
 *   (compute-heavy). It:
 *     1) blocks waiting for the run/wakeup semaphore; when signaled,
 *     2) performs one estimator iteration (predict + queued measurement updates + finalize)
 *     3) acquires the mutex for the simple snapshot, publishes the updated estimate, releases the mutex
 * 
 * Since estimatorKalman is faster than kalmanTask, the copied state doesn't change every time
 * estimatorKalman is called, but only when kalmanTask has completed another iteration.

 * ## Task entrypoint visibility: why `kalmanTask()` is `static`
 *
 * The FreeRTOS worker that runs the EKF is implemented as the file-local function
 * `kalmanTask()`. It is declared `static` to give it file-only visibility: no other 
 * translation unit can call it, it is not part of the public estimator API. 
 * Explicit that the only intended caller is the RTOS scheduler.
 *
 * Summary of roles:
 *  - estimatorKalmanTaskInit(): creates RTOS primitives and starts kalmanTask()
 *  - kalmanTask(): RTOS-owned worker loop (private; static).  Is the *producer worker* (compute + publish)
 *  - estimatorKalman(): stabilizer-facing hook (public API) that reads the
 *    published state and triggers the worker
 */

#include "kalman_core.h"
#include "kalman_core_params_defaults.h"
#include "kalman_supervisor.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"
#include "supervisor.h"
#include "axis3fSubSampler.h"
#include "deck.h"

#include "statsCnt.h"
#include "rateSupervisor.h"

// Measurement models
#include "mm_distance.h"
#include "mm_absolute_height.h"
#include "mm_position.h"
#include "mm_pose.h"
#include "mm_tdoa.h"
#include "mm_flow.h"
#include "mm_tof.h"
#include "mm_yaw_error.h"
#include "mm_sweep_angles.h"

#include "mm_tdoa_robust.h"
#include "mm_distance_robust.h"

#define DEBUG_MODULE "ESTKALMAN"
#include "debug.h"
#include "cfassert.h"


// #define KALMAN_USE_BARO_UPDATE


// Semaphore to signal that we got data from the stabilizer loop to process
static SemaphoreHandle_t kalmanTaskSignal;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t publishedEstimatorStateMutex;
static StaticSemaphore_t publishedEstimatorStateMutexBuffer;


/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 1000Hz
const uint32_t PREDICTION_UPDATE_INTERVAL_MS = 1000 / PREDICT_RATE;

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// Use the robust implementations of TWR and TDoA, off by default but can be turned on through a parameter.
// The robust implementations use around 10% more CPU VS the standard flavours
static bool robustTwr = false;
static bool robustTdoa = false;

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

 /**
  * kalmanCoreData_t coreData is the INTERNAL EKF state used by the Kalman task. It evolves during kalmanTask()
  * - Has state vector (position, velocity-ish terms, attitude error, biases/auxiliary states), 
  *  covariance matrix, rotation matrix and quaternion.
  */
NO_DMA_CCM_SAFE_ZERO_INIT static kalmanCoreData_t coreData;

/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */

static bool isInit = false;

static Axis3fSubSampler_t accSubSampler;
static Axis3fSubSampler_t gyroSubSampler;
static Axis3f accLatest;
static Axis3f gyroLatest;

static OutlierFilterTdoaState_t outlierFilterTdoaState;
static OutlierFilterLhState_t sweepOutlierFilterState;


// Indicates that the internal state is corrupt and should be reset
bool resetEstimation = false;

static kalmanCoreParams_t coreParams = {
  KALMAN_CORE_DEFAULT_PARAMS_INIT
};

// Data used to enable the task and stabilizer loop to run with minimal locking
/**
 * publishedEstimatorState is a simplified view of coreData.
 * publishedEstimatorState update happens at the end of each kalmanTask() iteration
 */
static state_t publishedEstimatorState;

// Statistics
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(predictionCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(finalizeCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);

static rateSupervisor_t rateSupervisorContext;

#define WARNING_HOLD_BACK_TIME_MS 2000
static uint32_t warningBlockTimeMs = 0;

#ifdef KALMAN_USE_BARO_UPDATE
static const bool useBaroUpdate = true;
#else
static const bool useBaroUpdate = false;
#endif

static void kalmanTask(void* parameters);
static void updateQueuedMeasurements(const uint32_t nowMs, const bool quadIsFlying);

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(kalmanTask, KALMAN_TASK_STACKSIZE);

// --------------------------------------------------

/**
 * @brief Allocate and start the Kalman estimator FreeRTOS task.
 *
 * - Creates the semaphore used to trigger the task from the stabilizer loop
 * - Initializes the mutex that protects @ref state_t 
 * - Spins up the @ref kalmanTask() worker.
 */
void estimatorKalmanTaskInit() {
  // It would be logical to set the params->attitudeReversion here, based on deck requirements, but the decks are
  // not initialized yet at this point so it is done in estimatorKalmanInit().

  // Created in the 'empty' state, meaning the semaphore must first be given, that is it will block in the task
  // until released by the stabilizer loop
  kalmanTaskSignal = xSemaphoreCreateBinary();
  ASSERT(kalmanTaskSignal);

  publishedEstimatorStateMutex = xSemaphoreCreateMutexStatic(&publishedEstimatorStateMutexBuffer);

  STATIC_MEM_TASK_CREATE(kalmanTask, kalmanTask, KALMAN_TASK_NAME, NULL, KALMAN_TASK_PRI);

  isInit = true;
}

/**
 * @brief Return true once the Kalman task and synchronization primitives exist.
 */
bool estimatorKalmanTaskTest() {
  return isInit;
}

/**
 * @brief Main EKF loop that runs in its own FreeRTOS task context.
 *
 * Waits for the stabilizer loop to post a semaphore, then:
 *  1. Handles reset requests (from params or supervisor).
 *  2. Finalizes IMU subsampling windows and runs the predict step at 100 Hz.
 *  3. Adds process noise every loop even if predict is skipped.
 *  4. Flushes the measurement queue, forwarding each packet to its measurement model.
 *  5. Finalizes the state (attitude error injection) and checks supervisor bounds.
 *  6. Copies the internal state to the shared @ref publishedEstimatorState.
 *  7. Loops back to wait for the next semaphore from the stabilizer.
 *
 * The function runs forever and is latency sensitive; locking is minimized to the
 * brief critical section that copies the state for the stabilizer loop.
 *
 * @param parameters Unused FreeRTOS task parameter.
 */
static void kalmanTask(void* parameters) {
  systemWaitStart();

  uint32_t nowMs = T2M(xTaskGetTickCount());
  uint32_t nextPredictionMs = nowMs;

  rateSupervisorInit(&rateSupervisorContext, nowMs, ONE_SECOND, PREDICT_RATE - 1, PREDICT_RATE + 1, 1);

  while (true) {
    xSemaphoreTake(kalmanTaskSignal, portMAX_DELAY);
    nowMs = T2M(xTaskGetTickCount()); // would be nice if this had a precision higher than 1ms...

    if (resetEstimation) {
      estimatorKalmanInit();
      resetEstimation = false;
    }

    #ifdef CONFIG_ESTIMATOR_KALMAN_GENERAL_PURPOSE
    bool quadIsFlying = false;
    #else
    bool quadIsFlying = supervisorIsFlying();
    #endif

    #ifdef KALMAN_DECOUPLE_XY
      kalmanCoreDecoupleXY(&coreData);
    #endif

    // --- Predict step (100 Hz) ---
    // IMU samples are integrated at a controlled rate using subsampled data.
    if (nowMs >= nextPredictionMs) {
      axis3fSubSamplerFinalize(&accSubSampler);
      axis3fSubSamplerFinalize(&gyroSubSampler);

      kalmanCorePredict(&coreData, &coreParams, &accSubSampler.subSample, &gyroSubSampler.subSample, nowMs, quadIsFlying);
      nextPredictionMs = nowMs + PREDICTION_UPDATE_INTERVAL_MS;

      STATS_CNT_RATE_EVENT(&predictionCounter);

      if (!rateSupervisorValidate(&rateSupervisorContext, nowMs)) {
        DEBUG_PRINT("WARNING: Kalman prediction rate off (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
      }
    }

    // --- Process noise integration ---
    // Keeps covariance growth tied to the actual loop frequency.
    kalmanCoreAddProcessNoise(&coreData, &coreParams, nowMs);

    // --- Measurement queue flush ---
    // Consume every measurement enqueued since the previous stabilizer loop.
    updateQueuedMeasurements(nowMs, quadIsFlying);

    // --- Finalize (attitude error -> quaternion) if any update happened ---
    if (kalmanCoreFinalize(&coreData))
    {
      STATS_CNT_RATE_EVENT(&finalizeCounter);
    }

    // --- Supervisor guard rails ---
    if (! kalmanSupervisorIsStateWithinBounds(&coreData)) {
      resetEstimation = true;

      if (nowMs > warningBlockTimeMs) {
        warningBlockTimeMs = nowMs + WARNING_HOLD_BACK_TIME_MS;
        DEBUG_PRINT("State out of bounds, resetting\n");
      }
    }

    // --- Externalize the current estimate from coreData to publishedEstimatorState for the stabilizer loop ---
    // The shared state includes body-frame acceleration, so we update every loop.
    // Note: the mutex is used to WRITE to publishedEstimatorState, while the stabilizer (estimatorKalman()) READS from it.
    xSemaphoreTake(publishedEstimatorStateMutex, portMAX_DELAY);
    kalmanCoreExternalizeState(&coreData, &publishedEstimatorState, &accLatest);
    xSemaphoreGive(publishedEstimatorStateMutex);

    STATS_CNT_RATE_EVENT(&updateCounter);
  }
}

/**
 * @brief Stabilizer hook that reads the most recent EKF state and wakes up kalmanTask().
 * 
 * Note: this is a consumer of publishedEstimatorState, while kalmanTask() is a producer.
 * 
 * Note: the stabilizer loop runs faster (500 Hz) than the Kalman predict step (100 Hz).
 * This is to ensure that the stabilizer always has fresh state to work with, even if 
 * the Kalman task is delayed for some reason. The Kalman task will simply skip predict
 * steps as needed to catch up. That's also why estimatorKalman() and kalmanTask() are 
 * separated by a semaphore.
 *
 * @param state Output pointer filled with the current state estimate.
 * @param stabilizerStep Provides the caller's timing context (unused here).
 */
void estimatorKalman(state_t *state, const stabilizerStep_t stabilizerStep) {
  // This function is called from the stabilizer loop. It is important that this call returns
  // as quickly as possible. The publishedEstimatorStateMutex must only be locked short periods by the task.
  // Note: the mutex is used to READ from publishedEstimatorState, while the kalman task WRITES to it.
  xSemaphoreTake(publishedEstimatorStateMutex, portMAX_DELAY);

  // Copy the latest state, calculated by the task
  memcpy(state, &publishedEstimatorState, sizeof(state_t));
  xSemaphoreGive(publishedEstimatorStateMutex);

  xSemaphoreGive(kalmanTaskSignal);
}

/**
 * @brief Drain the measurement queue and run the associated measurement models.
 *
 * Called once per Kalman task loop after the process noise update. Each packet is
 * dispatched based on @ref MeasurementType and handed off to the specialized
 * @c kalmanCoreUpdateWith* function. Gating/outlier logic lives inside each model.
 *
 * @param nowMs Current timestamp [ms]; passed down to models that need it (e.g. TDoA).
 * @param quadIsFlying Thrust flag to forward to barometer handling.
 */
static void updateQueuedMeasurements(const uint32_t nowMs, const bool quadIsFlying) {
  /**
   * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
   * we therefore consume all measurements since the last loop, rather than accumulating
   */

  // Pull the latest sensors values of interest; discard the rest
  measurement_t m;
  while (estimatorDequeue(&m)) {
    switch (m.type) {
      case MeasurementTypeTDOA:
        if(robustTdoa){
          // robust KF update with TDOA measurements
          kalmanCoreRobustUpdateWithTdoa(&coreData, &m.data.tdoa, &outlierFilterTdoaState);
        }else{
          // standard KF update
          kalmanCoreUpdateWithTdoa(&coreData, &m.data.tdoa, nowMs, &outlierFilterTdoaState);
        }
        break;
      case MeasurementTypePosition:
        kalmanCoreUpdateWithPosition(&coreData, &m.data.position);
        break;
      case MeasurementTypePose:
        kalmanCoreUpdateWithPose(&coreData, &m.data.pose);
        break;
      case MeasurementTypeDistance:
        if(robustTwr){
            // robust KF update with UWB TWR measurements
            kalmanCoreRobustUpdateWithDistance(&coreData, &m.data.distance);
        }else{
            // standard KF update
            kalmanCoreUpdateWithDistance(&coreData, &m.data.distance);
        }
        break;
      case MeasurementTypeTOF:
        kalmanCoreUpdateWithTof(&coreData, &m.data.tof);
        break;
      case MeasurementTypeAbsoluteHeight:
        kalmanCoreUpdateWithAbsoluteHeight(&coreData, &m.data.height);
        break;
      case MeasurementTypeFlow:
        kalmanCoreUpdateWithFlow(&coreData, &m.data.flow, &gyroLatest);
        break;
      case MeasurementTypeYawError:
        kalmanCoreUpdateWithYawError(&coreData, &m.data.yawError);
        break;
      case MeasurementTypeSweepAngle:
        kalmanCoreUpdateWithSweepAngles(&coreData, &m.data.sweepAngle, nowMs, &sweepOutlierFilterState);
        break;
      case MeasurementTypeGyroscope:
        axis3fSubSamplerAccumulate(&gyroSubSampler, &m.data.gyroscope.gyro);
        gyroLatest = m.data.gyroscope.gyro;
        break;
      case MeasurementTypeAcceleration:
        axis3fSubSamplerAccumulate(&accSubSampler, &m.data.acceleration.acc);
        accLatest = m.data.acceleration.acc;
        break;
      case MeasurementTypeBarometer:
        if (useBaroUpdate) {
          kalmanCoreUpdateWithBaro(&coreData, &coreParams, m.data.barometer.baro.asl, quadIsFlying);
        }
        break;
      default:
        break;
    }
  }
}

/**
 * @brief Initialize or reset the Kalman core and measurement filters.
 *
 * Called once when the estimator becomes active and again whenever a reset is
 * requested (parameter or supervisor). Re-configures deck-specific parameters,
 * clears outlier filters, resets the IMU sub-samplers and calls @ref kalmanCoreInit().
 */
void estimatorKalmanInit(void)
{
  #ifdef CONFIG_DECK_LOCO_2D_POSITION
  coreParams.attitudeReversion = 0.0f;
  #else
  if (deckGetRequiredKalmanEstimatorAttitudeReversionOff())
  {
    coreParams.attitudeReversion = 0.0f;
    DEBUG_PRINT("Attitude reversion deactivated by deck\n");
  }
  #endif

  axis3fSubSamplerInit(&accSubSampler, GRAVITY_MAGNITUDE);
  axis3fSubSamplerInit(&gyroSubSampler, DEG_TO_RAD);

  outlierFilterTdoaReset(&outlierFilterTdoaState);
  outlierFilterLighthouseReset(&sweepOutlierFilterState, 0);

  uint32_t nowMs = T2M(xTaskGetTickCount());
  kalmanCoreInit(&coreData, &coreParams, nowMs); // see kalman_core.[h,c]. Initializes coreData
}

/**
 * @brief Report whether the Kalman estimator task infrastructure is up.
 */
bool estimatorKalmanTest(void)
{
  return isInit;
}

/**
 * @brief Convenience accessor returning the latest world-frame XYZ position.
 */
void estimatorKalmanGetEstimatedPos(point_t* pos) {
  pos->x = coreData.S[KC_STATE_X];
  pos->y = coreData.S[KC_STATE_Y];
  pos->z = coreData.S[KC_STATE_Z];
}

/**
 * @brief Copy the 3x3 attitude rotation matrix from the core state.
 *
 * @param rotationMatrix Caller-provided buffer with room for 9 floats (row-major).
 */
void estimatorKalmanGetEstimatedRot(float * rotationMatrix) {
  memcpy(rotationMatrix, coreData.R, 9*sizeof(float));
}

/**
 * Variables and results from the Extended Kalman Filter
 */
LOG_GROUP_START(kalman)
 /**
 * @brief State position in the global frame x
 *
 *   Note: This is similar to stateEstimate.x.
 */
  LOG_ADD(LOG_FLOAT, stateX, &coreData.S[KC_STATE_X])
 /**
 * @brief State position in the global frame y
 *
 *  Note: This is similar to stateEstimate.y
 */
  LOG_ADD(LOG_FLOAT, stateY, &coreData.S[KC_STATE_Y])
 /**
 * @brief State position in the global frame z
 *
 *  Note: This is similar to stateEstimate.z
 */
  LOG_ADD(LOG_FLOAT, stateZ, &coreData.S[KC_STATE_Z])
  /**
  * @brief State velocity in its body frame x
  *
  *  Note: This should be part of stateEstimate
  */
  LOG_ADD(LOG_FLOAT, statePX, &coreData.S[KC_STATE_PX])
  /**
  * @brief State velocity in its body frame y
  *
  *  Note: This should be part of stateEstimate
  */
  LOG_ADD(LOG_FLOAT, statePY, &coreData.S[KC_STATE_PY])
  /**
  * @brief State velocity in its body frame z
  *
  *  Note: This should be part of stateEstimate
  */
  LOG_ADD(LOG_FLOAT, statePZ, &coreData.S[KC_STATE_PZ])
  /**
  * @brief State attitude error roll
  */
  LOG_ADD(LOG_FLOAT, stateD0, &coreData.S[KC_STATE_D0])
  /**
  * @brief State attitude error pitch
  */
  LOG_ADD(LOG_FLOAT, stateD1, &coreData.S[KC_STATE_D1])
  /**
  * @brief State attitude error yaw
  */
  LOG_ADD(LOG_FLOAT, stateD2, &coreData.S[KC_STATE_D2])
  /**
  * @brief Covariance matrix position x
  */
  LOG_ADD(LOG_FLOAT, varX, &coreData.P[KC_STATE_X][KC_STATE_X])
  /**
  * @brief Covariance matrix position y
  */
  LOG_ADD(LOG_FLOAT, varY, &coreData.P[KC_STATE_Y][KC_STATE_Y])
  /**
  * @brief Covariance matrix position z
  */
  LOG_ADD(LOG_FLOAT, varZ, &coreData.P[KC_STATE_Z][KC_STATE_Z])
  /**
  * @brief Covariance matrix velocity x
  */
  LOG_ADD(LOG_FLOAT, varPX, &coreData.P[KC_STATE_PX][KC_STATE_PX])
  /**
  * @brief Covariance matrix velocity y
  */
  LOG_ADD(LOG_FLOAT, varPY, &coreData.P[KC_STATE_PY][KC_STATE_PY])
  /**
  * @brief Covariance matrix velocity z
  */
  LOG_ADD(LOG_FLOAT, varPZ, &coreData.P[KC_STATE_PZ][KC_STATE_PZ])
  /**
  * @brief Covariance matrix attitude error roll
  */
  LOG_ADD(LOG_FLOAT, varD0, &coreData.P[KC_STATE_D0][KC_STATE_D0])
  /**
  * @brief Covariance matrix attitude error pitch
  */
  LOG_ADD(LOG_FLOAT, varD1, &coreData.P[KC_STATE_D1][KC_STATE_D1])
  /**
  * @brief Covariance matrix attitude error yaw
  */
  LOG_ADD(LOG_FLOAT, varD2, &coreData.P[KC_STATE_D2][KC_STATE_D2])
  /**
  * @brief Estimated Attitude quarternion w
  */
  LOG_ADD(LOG_FLOAT, q0, &coreData.q[0])
  /**
  * @brief Estimated Attitude quarternion x
  */
  LOG_ADD(LOG_FLOAT, q1, &coreData.q[1])
  /**
  * @brief Estimated Attitude quarternion y
  */
  LOG_ADD(LOG_FLOAT, q2, &coreData.q[2])
  /**
  * @brief Estimated Attitude quarternion z
  */
  LOG_ADD(LOG_FLOAT, q3, &coreData.q[3])
  /**
  * @brief Statistics rate of update step
  */
  STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
  /**
  * @brief Statistics rate of prediction step
  */
  STATS_CNT_RATE_LOG_ADD(rtPred, &predictionCounter)
  /**
  * @brief Statistics rate full estimation step
  */
  STATS_CNT_RATE_LOG_ADD(rtFinal, &finalizeCounter)
LOG_GROUP_STOP(kalman)

LOG_GROUP_START(outlierf)
  LOG_ADD(LOG_INT32, lhWin, &sweepOutlierFilterState.openingWindowMs)
LOG_GROUP_STOP(outlierf)

/**
 * Tuning parameters for the Extended Kalman Filter (EKF)
 *     estimator
 */
PARAM_GROUP_START(kalman)
/**
 * @brief Reset the kalman estimator
 */
  PARAM_ADD_CORE(PARAM_UINT8, resetEstimation, &resetEstimation)
/**
 * @brief Nonzero to use robust TDOA method (default: 0)
 */
  PARAM_ADD_CORE(PARAM_UINT8, robustTdoa, &robustTdoa)
/**
 * @brief Nonzero to use robust TWR method (default: 0)
 */
  PARAM_ADD_CORE(PARAM_UINT8, robustTwr, &robustTwr)
/**
 * @brief Process noise for x and y acceleration
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAcc_xy, &coreParams.procNoiseAcc_xy)
/**
 * @brief Process noise for z acceleration
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAcc_z, &coreParams.procNoiseAcc_z)
  /**
 * @brief Process noise for velocity
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNVel, &coreParams.procNoiseVel)
  /**
 * @brief Process noise for position
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNPos, &coreParams.procNoisePos)
  /**
 * @brief Process noise for attitude
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAtt, &coreParams.procNoiseAtt)
  /**
 * @brief Measurement noise for barometer
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNBaro, &coreParams.measNoiseBaro)
  /**
 * @brief Measurement noise for roll/pitch gyros
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNGyro_rollpitch, &coreParams.measNoiseGyro_rollpitch)
  /**
 * @brief Measurement noise for yaw gyro
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNGyro_yaw, &coreParams.measNoiseGyro_yaw)
  /**
 * @brief Initial X after reset [m]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialX, &coreParams.initialX)
  /**
 * @brief Initial Y after reset [m]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialY, &coreParams.initialY)
  /**
 * @brief Initial Z after reset [m]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialZ, &coreParams.initialZ)
  /**
 * @brief Initial yaw after reset [rad]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialYaw, &coreParams.initialYaw)
PARAM_GROUP_STOP(kalman)
