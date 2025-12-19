/**
 * @file estimator.c
 * @brief Estimator front-end that multiplexes between implementations and funnels sensor data.
 *
 * Responsibilities:
 * - Own the measurement queue shared between sensor/deck drivers and the active estimator.
 * - Provide the `stateEstimator*` API used by the stabilizer task to initialize, switch and run
 *   complementary, Kalman, UKF or out-of-tree estimators.
 * - Expose helper enqueue/dequeue functions that let any producer push `measurement_t` packets
 *   without knowing which backend is selected.
 * - Emit eventtrigger hooks and statistics for logging/monitoring when measurements arrive or
 *   are dropped.
 *
 * Directory context: files under `src/modules/src/estimator/` share this interface. Each concrete
 * estimator (e.g. `estimator_kalman.c`) implements the math/RTOS behavior, while this file provides
 * the glue that keeps the system running regardless of which estimator is enabled.
 */

#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"

#define DEBUG_MODULE "ESTIMATOR"
#include "debug.h"

#include "cfassert.h"
#include "estimator.h"
#include "estimator_complementary.h"
#include "estimator_kalman.h"
#include "estimator_ukf.h"
#include "log.h"
#include "statsCnt.h"
#include "eventtrigger.h"
#include "quatcompress.h"

#define DEFAULT_ESTIMATOR StateEstimatorTypeComplementary
static StateEstimatorType currentEstimator = StateEstimatorTypeAutoSelect;


#define MEASUREMENTS_QUEUE_SIZE (20)
static xQueueHandle measurementsQueue;
STATIC_MEM_QUEUE_ALLOC(measurementsQueue, MEASUREMENTS_QUEUE_SIZE, sizeof(measurement_t));

// Statistics
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);

// events
EVENTTRIGGER(estTDOA, uint8, idA, uint8, idB, float, distanceDiff)
EVENTTRIGGER(estPosition, uint8, source)
EVENTTRIGGER(estPose)
EVENTTRIGGER(estDistance, uint8, id, float, distance)
EVENTTRIGGER(estTOF)
EVENTTRIGGER(estAbsoluteHeight)
EVENTTRIGGER(estFlow)
EVENTTRIGGER(estYawError, float, yawError)
EVENTTRIGGER(estSweepAngle, uint8, sensorId, uint8, baseStationId, uint8, sweepId, float, t, float, sweepAngle)
EVENTTRIGGER(estGyroscope)
EVENTTRIGGER(estAcceleration)
EVENTTRIGGER(estBarometer)

static void initEstimator(const StateEstimatorType estimator);
static void deinitEstimator(const StateEstimatorType estimator);

typedef struct {
  void (*init)(void);
  void (*deinit)(void);
  bool (*test)(void);
  void (*update)(state_t *state, const stabilizerStep_t stabilizerStep);
  const char* name;
} EstimatorFcns;

#define NOT_IMPLEMENTED ((void*)0)

/**
 * Note: it follows the order of StateEstimatorType enum
 * 
 */
static EstimatorFcns estimatorFunctions[] = {
    {
        .init = NOT_IMPLEMENTED,
        .deinit = NOT_IMPLEMENTED,
        .test = NOT_IMPLEMENTED,
        .update = NOT_IMPLEMENTED,
        .name = "None",
    }, // Any estimator
    {
        .init = estimatorComplementaryInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorComplementaryTest,
        .update = estimatorComplementary,
        .name = "Complementary",
    },
#ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
    {
        .init = estimatorKalmanInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorKalmanTest,
        .update = estimatorKalman,
        .name = "Kalman",
    },
#endif
#ifdef CONFIG_ESTIMATOR_UKF_ENABLE
    {
	    .init = errorEstimatorUkfInit,
	    .deinit = NOT_IMPLEMENTED,
	    .test = errorEstimatorUkfTest,
	    .update = errorEstimatorUkf,
	    .name = "Error State UKF",
	},
#endif
#ifdef CONFIG_ESTIMATOR_OOT
    {
        .init = estimatorOutOfTreeInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorOutOfTreeTest,
        .update = estimatorOutOfTree,
        .name = "OutOfTree",
    },
#endif
};

/**
 * @brief Create the measurement queue and start the requested estimator backend.
 *
 * Called during system init from the stabilizer code path.
 *
 * @param estimator Requested estimator type (auto-select resolves to default build option).
 */
void stateEstimatorInit(StateEstimatorType estimator) {
  measurementsQueue = STATIC_MEM_QUEUE_CREATE(measurementsQueue);
  stateEstimatorSwitchTo(estimator);
}

/**
 * @brief Switch to a new estimator implementation at runtime.
 *
 * Deinitializes the previous backend, respects build-time forced estimators and
 * keeps `currentEstimator` updated.
 *
 * @param estimator Desired estimator type or auto-select.
 */
void stateEstimatorSwitchTo(StateEstimatorType estimator) {
  if (estimator < 0 || estimator >= StateEstimatorType_COUNT) {
    return;
  }

  StateEstimatorType newEstimator = estimator;

  if (StateEstimatorTypeAutoSelect == newEstimator) {
    newEstimator = DEFAULT_ESTIMATOR;
  }

  #if defined(CONFIG_ESTIMATOR_KALMAN)
    #define ESTIMATOR StateEstimatorTypeKalman
  #elif defined(CONFIG_ESTIMATOR_UKF)
    #define ESTIMATOR StateEstimatorTypeUkf
  #elif defined(CONFIG_ESTIMATOR_COMPLEMENTARY)
    #define ESTIMATOR StateEstimatorTypeComplementary
  #else
    #define ESTIMATOR StateEstimatorTypeAutoSelect
  #endif

  StateEstimatorType forcedEstimator = ESTIMATOR;
  if (forcedEstimator != StateEstimatorTypeAutoSelect) {
    DEBUG_PRINT("Estimator type forced\n");
    newEstimator = forcedEstimator;
  }

  initEstimator(newEstimator);
  StateEstimatorType previousEstimator = currentEstimator;
  currentEstimator = newEstimator;
  deinitEstimator(previousEstimator);

  DEBUG_PRINT("Using %s (%d) estimator\n", stateEstimatorGetName(), currentEstimator);
}

/**
 * @brief Return the currently active estimator type.
 */
StateEstimatorType stateEstimatorGetType(void) {
  return currentEstimator;
}

static void initEstimator(const StateEstimatorType estimator) {
  if (estimatorFunctions[estimator].init) {
    estimatorFunctions[estimator].init();
  }
}

static void deinitEstimator(const StateEstimatorType estimator) {
  if (estimatorFunctions[estimator].deinit) {
    estimatorFunctions[estimator].deinit();
  }
}

/**
 * @brief Run the active estimator's self-test hook.
 */
bool stateEstimatorTest(void) {
  return estimatorFunctions[currentEstimator].test();
}

/**
 * @brief Invoke the active estimator update function.
 *
 * Called from the stabilizer loop once per control step.
 */
void stateEstimator(state_t *state, const stabilizerStep_t tick) {
  estimatorFunctions[currentEstimator].update(state, tick);
}

/**
 * @brief Human-readable name of the active estimator.
 */
const char* stateEstimatorGetName() {
  return estimatorFunctions[currentEstimator].name;
}


/**
 * @brief Push a measurement into the shared queue.
 *
 * Thread-safe helper used by drivers and tasks. Dispatches to the correct FreeRTOS
 * queue API depending on context and updates stats/event triggers.
 *
 * @param measurement Measurement packet to enqueue.
 */
void estimatorEnqueue(const measurement_t *measurement) {
  if (!measurementsQueue) {
    return;
  }

  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(measurementsQueue, measurement, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
      portYIELD();
    }
  } else {
    result = xQueueSend(measurementsQueue, measurement, 0);
  }

  if (result == pdTRUE) {
    STATS_CNT_RATE_EVENT(&measurementAppendedCounter);
  } else {
    STATS_CNT_RATE_EVENT(&measurementNotAppendedCounter);
  }

  // events
  switch (measurement->type) {
    case MeasurementTypeTDOA:
      eventTrigger_estTDOA_payload.idA = measurement->data.tdoa.anchorIds[0];
      eventTrigger_estTDOA_payload.idB = measurement->data.tdoa.anchorIds[1];
      eventTrigger_estTDOA_payload.distanceDiff = measurement->data.tdoa.distanceDiff;
      eventTrigger(&eventTrigger_estTDOA);
      break;
    case MeasurementTypePosition:
      // for additional data, see locSrv.{x,y,z} and lighthouse.{x,y,z}
      eventTrigger_estPosition_payload.source = measurement->data.position.source;
      eventTrigger(&eventTrigger_estPosition);
      break;
    case MeasurementTypePose:
      // no payload needed, see locSrv.{x,y,z,qx,qy,qz,qw}
      eventTrigger(&eventTrigger_estPose);
      break;
    case MeasurementTypeDistance:
      eventTrigger_estDistance_payload.id = measurement->data.distance.anchorId;
      eventTrigger_estDistance_payload.distance = measurement->data.distance.distance;
      eventTrigger(&eventTrigger_estDistance);
      break;
    case MeasurementTypeTOF:
      // no payload needed, see range.zrange
      eventTrigger(&eventTrigger_estTOF);
      break;
    case MeasurementTypeAbsoluteHeight:
      // no payload needed, see CONFIG_DECK_LOCO_2D_POSITION
      eventTrigger(&eventTrigger_estAbsoluteHeight);
      break;
    case MeasurementTypeFlow:
      // no payload needed, see motion.{deltaX,deltaY}
      eventTrigger(&eventTrigger_estFlow);
      break;
    case MeasurementTypeYawError:
      eventTrigger_estYawError_payload.yawError = measurement->data.yawError.yawError;
      eventTrigger(&eventTrigger_estYawError);
      break;
    case MeasurementTypeSweepAngle:
      eventTrigger_estSweepAngle_payload.sensorId = measurement->data.sweepAngle.sensorId;
      eventTrigger_estSweepAngle_payload.baseStationId = measurement->data.sweepAngle.baseStationId;
      eventTrigger_estSweepAngle_payload.sweepId = measurement->data.sweepAngle.sweepId;
      eventTrigger_estSweepAngle_payload.t = measurement->data.sweepAngle.t;
      eventTrigger_estSweepAngle_payload.sweepAngle = measurement->data.sweepAngle.measuredSweepAngle;
      eventTrigger(&eventTrigger_estSweepAngle);
      break;
    case MeasurementTypeGyroscope:
      // no payload needed, see gyro.{x,y,z}
      eventTrigger(&eventTrigger_estGyroscope);
      break;
    case MeasurementTypeAcceleration:
      // no payload needed, see acc.{x,y,z}
      eventTrigger(&eventTrigger_estAcceleration);
      break;
    case MeasurementTypeBarometer:
      // no payload needed, see baro.asl
      eventTrigger(&eventTrigger_estBarometer);
      break;
    default:
      break;
  }
}

/**
 * @brief Pop the oldest measurement from the queue (non-blocking).
 *
 * Called from estimator implementations (`estimator_kalman.c`) inside their worker tasks.
 *
 * @param measurement Output pointer filled if an entry is available.
 * @return true when a measurement was dequeued.
 */
bool estimatorDequeue(measurement_t *measurement) {
  return pdTRUE == xQueueReceive(measurementsQueue, measurement, 0);
}

LOG_GROUP_START(estimator)
  STATS_CNT_RATE_LOG_ADD(rtApnd, &measurementAppendedCounter)
  STATS_CNT_RATE_LOG_ADD(rtRej, &measurementNotAppendedCounter)
LOG_GROUP_STOP(estimator)
