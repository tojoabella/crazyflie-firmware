# Kalman Filter Pipeline: Sensors → Estimator → Controller

This document describes the complete data flow through the Crazyflie Kalman filter, from raw sensor measurements to state estimates consumed by the flight controller.

## Overview

The Kalman estimator pipeline is designed as a **multi-threaded, asynchronous measurement-driven architecture**:

1. **Sensor tasks/ISRs** produce raw measurements at various rates (1 kHz IMU, ~100 Hz flow, variable rate LPS/Lighthouse)
2. **Measurement queue** (`estimator.c`) buffers measurements as a `measurement_t` union, decoupling sensor production from estimator consumption
3. **Kalman task** (`estimatorKalmanTask`) runs as a FreeRTOS task, dequeuing measurements, running predict/update cycles, and exporting state
4. **Stabilizer loop** (`stabilizerTask`) runs at 1 kHz, reading the latest exported state for control feedback

**Key Design Principles**:
- **Non-blocking**: Stabilizer loop never blocks on estimator; it reads the latest available state
- **Asynchronous updates**: Measurements arrive asynchronously; Kalman task processes them as fast as possible
- **Sensor agnostic**: Deck drivers enqueue measurements via common API; estimator is unaware of sensor hardware details

## High-Level Call Graph

### Stabilizer Loop (Main Control Thread)

```
stabilizerTask()  [1 kHz, runs in FreeRTOS task "STAB"]
  └─ sensorsWaitDataReady()              # Blocks until IMU data ready (~1 kHz)
  └─ sensorsAcquire(&sensorData)         # Read IMU, enqueue accel/gyro measurements
  └─ stateEstimator(&state, stabilizerStep)  # Get latest state estimate
       └─ estimatorKalman(&state, step)  # Dispatch to Kalman backend
            ├─ xSemaphoreTake(publishedEstimatorStateMutex)
            ├─ memcpy(state, &publishedEstimatorState)  # Copy latest state (non-blocking)
            ├─ xSemaphoreGive(publishedEstimatorStateMutex)
            └─ xSemaphoreGive(kalmanTaskSignal)    # Wake Kalman task
  └─ commanderGetSetpoint(&setpoint)     # Get desired position/velocity/attitude
  └─ controller(&control, &setpoint, &sensorData, &state)  # Run PID controller
  └─ powerDistribution(&control)         # Compute motor PWM commands
```

### Kalman Task (State Estimation Thread)

```
kalmanTask()  [FreeRTOS task "ESTM", runs ~250-500 Hz depending on measurement load]
  └─ xSemaphoreTake(kalmanTaskSignal)    # Wait for stabilizer to signal or timeout

  # 1. IMU Subsampling and Prediction (100 Hz)
  ├─ if (accel/gyro measurements in queue):
  │    └─ axis3fSubSamplerAccumulate(&accel/gyro)  # Accumulate 1 kHz IMU samples
  ├─ if (time for prediction, 10ms interval):
  │    ├─ axis3fSubSamplerFinalize(&accelAvg, &gyroAvg)  # Average 10 IMU samples
  │    └─ kalmanCorePredict(&coreData, &accelAvg, &gyroAvg, dt)  # EKF predict step

  # 2. Process Noise Injection
  ├─ kalmanCoreAddProcessNoise(&coreData, &params, dt)  # Inflate covariance P

  # 3. Measurement Updates (drain queue)
  ├─ updateQueuedMeasurements(&coreData)
  │    └─ while (estimatorDequeue(&measurement)):
  │         ├─ switch (measurement.type):
  │         ├─   case MeasurementTypeTDOA:
  │         │      └─ kalmanCoreUpdateWithTDOA(&coreData, &measurement.data.tdoa)
  │         ├─   case MeasurementTypePosition:
  │         │      └─ kalmanCoreUpdateWithPosition(&coreData, &measurement.data.position)
  │         ├─   case MeasurementTypeFlow:
  │         │      └─ kalmanCoreUpdateWithFlow(&coreData, &measurement.data.flow)
  │         └─   ... (all measurement types dispatched similarly)

  # 4. Finalize Attitude Error
  ├─ if (state was updated):
  │    └─ kalmanCoreFinalize(&coreData)  # Inject attitude error into quaternion

  # 5. Supervisor Safety Checks
  ├─ if (!kalmanSupervisorIsStateWithinBounds(&coreData)):
  │    └─ resetEstimation = true  # Trigger re-init on next iteration

  # 6. Export State to Stabilizer
  ├─ xSemaphoreTake(publishedEstimatorStateMutex)
  ├─ kalmanCoreExternalizeState(&coreData, &publishedEstimatorState)  # Copy to shared state
  └─ xSemaphoreGive(publishedEstimatorStateMutex)
```

**Summary**:
- `stabilizerTask()` (1 kHz) waits for `sensorsWaitDataReady()` and calls `stateEstimator()` once per loop.
- `stateEstimator()` dispatches to the active backend (`estimatorKalman()` when Kalman is selected via `stabilizer.estimator` parameter).
- `estimatorKalman()` copies the latest EKF state into the caller-provided `state_t` and wakes the Kalman task via semaphore.
- `kalmanTask()` (FreeRTOS task created by `estimatorKalmanTaskInit()`) performs predict, process-noise, measurement flush, finalize, supervisor guard, and externalization.
- `kalmanCore*()` functions do the heavy math: `kalmanCorePredict()` (100 Hz), `kalmanCoreAddProcessNoise()` (every iteration), `kalmanCoreUpdateWith*()` per measurement, and `kalmanCoreFinalize()` / `kalmanCoreExternalizeState()` before handing results back to the stabilizer loop.

---

## Detailed Loop Phases inside `kalmanTask()`

The Kalman task runs in a continuous loop, processing measurements and updating state. Each iteration consists of several distinct phases:

### Phase 1: IMU Aggregation and Subsampling

**Purpose**: Downsample 1 kHz IMU data to 100 Hz for EKF prediction, reducing computational load while preserving signal quality.

**Implementation** ([estimator_kalman.c](../../src/modules/src/estimator/estimator_kalman.c)):
```c
// When MeasurementTypeGyroscope or MeasurementTypeAcceleration arrives:
if (measurement.type == MeasurementTypeGyroscope) {
  axis3fSubSamplerAccumulate(&gyroSubSampler, &measurement.data.gyroscope.gyro);
}
if (measurement.type == MeasurementTypeAcceleration) {
  axis3fSubSamplerAccumulate(&accSubSampler, &measurement.data.acceleration.acc);
}

// Every 10ms (PREDICTION_UPDATE_INTERVAL_MS):
if (doPrediction) {
  axis3fSubSamplerFinalize(&gyroSubSampler, &gyroAvg);     // Average 10 gyro samples
  axis3fSubSamplerFinalize(&accSubSampler, &accAvg);       // Average 10 accel samples
  // ... proceed to prediction
}
```

**Notes**:
- IMU samples arrive at 1 kHz from `sensorsAcquire()` in stabilizer loop
- Subsampler accumulates samples and computes average when `Finalize()` is called
- Averaging reduces high-frequency noise without introducing delay (causal filter)
- Prediction rate is `PREDICT_RATE = 100 Hz` (defined in `estimator_kalman.c`)

---

### Phase 2: EKF Predict Step

**Purpose**: Integrate IMU measurements to propagate nominal state forward in time and inflate covariance to account for process noise.

**Implementation** ([kalman_core.c:kalmanCorePredict](../../src/modules/src/kalman_core/kalman_core.c)):
```c
void kalmanCorePredict(kalmanCoreData_t* this, Axis3f *acc, Axis3f *gyro, float dt)
```

**Steps**:
1. **Nominal state integration**:
   - Integrate gyro to update quaternion: `q_new = q_old + 0.5 * dt * q_old ⊗ [0, ωx, ωy, ωz]`
   - Transform accel to world frame: `a_world = R(q) * a_body + g`
   - Integrate accel to update velocity: `v_new = v_old + dt * a_world`
   - Integrate velocity to update position: `p_new = p_old + dt * v_new`

2. **Covariance propagation**:
   - Build continuous-time system matrix `A` (9×9) as function of current attitude and acceleration
   - Discretize: `F ≈ I + A*dt` (first-order approximation)
   - Propagate: `P = F*P*F^T + G*Q*G^T` where `Q` is process noise, `G` maps noise to states

**Timing**:
- Called at 100 Hz (`PREDICT_RATE`)
- Actual dt measured from timestamps for accuracy
- If prediction is late (dt > 1.2 * nominal), a rate warning is issued

**See also**: [STATE_AND_FRAMES.md](STATE_AND_FRAMES.md) for coordinate frame details.

---

### Phase 3: Process Noise Injection

**Purpose**: Inflate covariance `P` to account for unmodeled dynamics and sensor noise, even when no prediction occurs.

**Implementation** ([kalman_core.c:kalmanCoreAddProcessNoise](../../src/modules/src/kalman_core/kalman_core.c)):
```c
void kalmanCoreAddProcessNoise(kalmanCoreData_t* this, kalmanCoreParams_t* params, float dt)
```

**Steps**:
- Add continuous-time process noise scaled by `dt`:
  - Position variance: `P[0:2, 0:2] += (procNoiseAcc_xy² * dt³/3)` (from acceleration noise)
  - Velocity variance: `P[3:5, 3:5] += (procNoiseAcc_xy² * dt)` + optional velocity random walk
  - Attitude variance: `P[6:8, 6:8] += (procNoiseAtt² * dt)` (from gyro noise)

**Notes**:
- Called **every Kalman task iteration**, regardless of whether prediction occurred
- Ensures covariance grows monotonically with real time, preventing overconfidence
- Process noise parameters are tunable: `kalman.pNAcc_xy`, `kalman.pNAcc_z`, `kalman.pNVel`, `kalman.pNAtt`
- Higher noise when flying (`quadIsFlying=1`) to account for aerodynamic disturbances

---

### Phase 4: Measurement Queue Flush and Updates

**Purpose**: Process all queued measurements, updating EKF state and covariance for each valid observation.

**Implementation** ([estimator_kalman.c:updateQueuedMeasurements](../../src/modules/src/estimator/estimator_kalman.c)):
```c
static void updateQueuedMeasurements(kalmanCoreData_t* coreData) {
  measurement_t m;
  while (estimatorDequeue(&m)) {  // Drain queue
    switch (m.type) {
      case MeasurementTypeTDOA:
        if (robustTdoa) {
          kalmanCoreUpdateWithTdoaRobust(coreData, &m.data.tdoa, ...);
        } else {
          kalmanCoreUpdateWithTDOA(coreData, &m.data.tdoa, ...);
        }
        break;
      case MeasurementTypePosition:
        kalmanCoreUpdateWithPosition(coreData, &m.data.position, ...);
        break;
      // ... all measurement types
    }
  }
}
```

**Measurement Types Supported** (see [SENSORS.md](SENSORS.md) for details):
- `MeasurementTypeTDOA`: UWB time-difference-of-arrival constraints
- `MeasurementTypeDistance`: UWB two-way ranging distance
- `MeasurementTypePosition`: Absolute position (MoCap, external localization)
- `MeasurementTypePose`: Full pose (position + quaternion)
- `MeasurementTypeFlow`: Optical flow (pixel velocity)
- `MeasurementTypeTOF`: Time-of-flight range (VL53L0X/VL53L1X)
- `MeasurementTypeAbsoluteHeight`: Absolute height sensor
- `MeasurementTypeSweepAngle`: Lighthouse sweep angles (multiple sensors × multiple bases)
- `MeasurementTypeYawError`: Yaw error correction (from Lighthouse or magnetometer)
- `MeasurementTypeBarometer`: Barometer altitude (processed as absolute height)
- `MeasurementTypeGyroscope`: Gyro data (used for subsampling, not direct update)
- `MeasurementTypeAcceleration`: Accel data (used for subsampling, not direct update)

**Measurement Model Pattern**:
Each `kalmanCoreUpdateWith*()` function follows this pattern:
1. Compute predicted measurement: `h(x_nominal)`
2. Compute innovation: `y = z - h(x_nominal)`
3. Check gating/outlier filter: reject if `|y| > threshold` or chi-square test fails
4. Compute Jacobian: `H = ∂h/∂δx` (derivative w.r.t. error state)
5. Call `kalmanCoreScalarUpdate(...)` or `kalmanCoreUpdateWithPKE(...)` to update state and covariance

**Notes**:
- Measurements are processed **immediately** (not batched) to minimize latency
- Robust variants (`robustTdoa`, `robustTwr`) use M-estimation (IRLS) to downweight outliers
- Outlier filters may run **before** the measurement model (e.g., `outlierFilterTdoa`, `outlierFilterLighthouse`)

---

### Phase 5: Finalize Attitude Error

**Purpose**: Inject accumulated attitude error δθ into the nominal quaternion `q`, then reset error to zero.

**Implementation** ([kalman_core.c:kalmanCoreFinalize](../../src/modules/src/kalman_core/kalman_core.c)):
```c
void kalmanCoreFinalize(kalmanCoreData_t* this)
```

**Steps**:
1. Check if state was updated (flag set by predict or measurement update)
2. Convert attitude error δθ (3D axis-angle) to delta quaternion δq:
   - Small-angle approximation: `δq ≈ [1, δθx/2, δθy/2, δθz/2]`
   - Or exact: `δq = [cos(|δθ|/2), sin(|δθ|/2) * δθ/|δθ|]`
3. Apply to nominal quaternion: `q = δq ⊗ q` (quaternion multiplication)
4. Renormalize quaternion: `q = q / ||q||`
5. Reset attitude error: `δθ = [0, 0, 0]`
6. Recompute rotation matrix `R(q)` for next iteration

**Why This Matters**:
- Error-state formulation keeps error states small (good for linearization)
- Quaternion must remain unit norm for validity
- Separating nominal quaternion from error avoids singularities in covariance

---

### Phase 6: Supervisor Safety Checks

**Purpose**: Detect divergence or unrealistic estimates, trigger emergency recovery if bounds violated.

**Implementation** ([kalman_supervisor.c:kalmanSupervisorIsStateWithinBounds](../../src/modules/src/kalman_supervisor.c)):
```c
bool kalmanSupervisorIsStateWithinBounds(kalmanCoreData_t* coreData)
```

**Checks Performed**:
- **NaN detection**: Check for NaN in position, velocity, or quaternion
- **Position bounds**: `|position| < maxPos` (default ~100m, tunable via `kalSup.maxPos`)
- **Velocity bounds**: `|velocity| < maxVel` (default ~10m/s, tunable via `kalSup.maxVel`)
- **Covariance bounds**: Check that `P` diagonal elements are not unreasonably large (divergence indicator)

**Actions on Failure**:
- Set `resetEstimation = true`
- On next iteration, `estimatorKalmanInit()` is called to re-initialize EKF to safe defaults
- Supervisor state machine may trigger emergency landing or kill motors

**Notes**:
- Supervisor is **conservative**: false positives are acceptable to prevent crashes
- Bounds are configurable via parameters for different flight environments (indoor vs outdoor)

---

### Phase 7: Externalize State

**Purpose**: Convert internal EKF state to public `state_t` format for consumption by stabilizer loop.

**Implementation** ([kalman_core.c:kalmanCoreExternalizeState](../../src/modules/src/kalman_core/kalman_core.c)):
```c
void kalmanCoreExternalizeState(kalmanCoreData_t* coreData, state_t* state, Axis3f* accLatest)
```

**Exported Fields** (see [stabilizer_types.h:state_t](../../src/modules/interface/stabilizer_types.h)):
- `state->position` (3D, meters, world frame)
- `state->velocity` (3D, m/s, world frame)
- `state->acc` (3D, m/s², world frame - latest accel transformed from body frame)
- `state->attitude` (roll, pitch, yaw, radians, converted from quaternion)
- `state->attitudeQuaternion` (quaternion, world → body rotation)

**Thread Safety**:
- Protected by `publishedEstimatorStateMutex` semaphore
- Stabilizer loop reads this state via `estimatorKalman()` which also takes `publishedEstimatorStateMutex`
- Copy is fast (<1µs), no blocking concerns

---

## Measurement Queue and Threading

### Queue Design ([estimator.c](../../src/modules/src/estimator/estimator.c))

**Data Structure**:
- FreeRTOS queue (`xQueueCreate(ESTIMATOR_QUEUE_SIZE, sizeof(measurement_t))`)
- Queue size: typically 20-50 entries (build-time configurable)
- Stores `measurement_t` union (all measurement types)

**Producer Side** (Sensor drivers, ISRs, deck tasks):
```c
// Example: Enqueue TDoA measurement from LPS deck
tdoaMeasurement_t tdoaData = { .distanceDiff = ..., .anchorIds = {id0, id1}, ... };
estimatorEnqueueTDOA(&tdoaData);
  └─ xQueueSendToBack(estimatorQueue, &measurement, timeout)
```

**Consumer Side** (Kalman task):
```c
// Drain queue in kalmanTask
measurement_t m;
while (estimatorDequeue(&m)) {  // Non-blocking, returns false if queue empty
  // Process measurement...
}
```

**Key Properties**:
- **Non-blocking enqueue**: ISRs use `xQueueSendToBackFromISR`, tasks use short timeout
- **FIFO ordering**: Measurements processed in arrival order (important for time-correlated sensors)
- **Overflow handling**: If queue full, oldest measurement may be dropped (depends on implementation)
- **Statistics**: `estimator.c` tracks queue depth, overflow events via logging

---

### Threading Model

**Three Main Threads**:

1. **Stabilizer Task** (`STAB`, priority: high, 1 kHz):
   - Reads sensors, calls `stateEstimator()`, runs controller
   - **Critical path**: Must maintain 1 kHz for stable flight
   - Only reads latest state (non-blocking), never waits for estimator

2. **Kalman Task** (`ESTM`, priority: medium, ~250-500 Hz):
   - Processes measurements, runs EKF math
   - **CPU-intensive**: Matrix operations, Jacobian computations
   - Woken by stabilizer via semaphore or timeout (typically 2-4ms)

3. **Sensor Tasks / ISRs** (various priorities):
   - IMU ISR: Highest priority, enqueues accel/gyro at 1 kHz
   - Deck tasks: Medium priority, enqueue measurements asynchronously
   - **Fast path**: Enqueue only (no processing), return to caller quickly

**Synchronization**:
- `kalmanTaskSignal`: Stabilizer signals Kalman task to wake
- `publishedEstimatorStateMutex`: Protects shared `publishedEstimatorState` during read/write
- Queue: Implicit synchronization via FreeRTOS queue primitives

**Latency Analysis**:
- **Sensor to queue**: <100µs (ISR or task enqueue overhead)
- **Queue to update**: Variable (depends on queue depth, up to ~20ms if queue full)
- **Update to externalize**: <1ms (EKF math + finalize)
- **Externalize to control**: <1ms (stabilizer loop reads state)
- **Total latency**: Typically 2-5ms for high-rate sensors (IMU), 10-50ms for low-rate sensors (flow, Lighthouse)

## Measurement queue and threading
- `estimator.c` owns the FreeRTOS queue that buffers `measurement_t` structures. Sensor drivers call `estimatorEnqueue*()` (declared in `estimator.h`) which copies the payload, posts it to that queue (from task or ISR context) and emits statistics/event triggers.
- `stateEstimator()` (running in the stabilizer task) never blocks on sensors; it simply copies the latest state and wakes the Kalman task. All heavy lifting happens inside `kalmanTask()` where there is no control-loop latency pressure.
- `estimatorDequeue()` is called from `kalmanTask()` to drain the queue. Measurements are not accumulated per-type; every entry is consumed on the next iteration to minimize latency between sensors and the EKF.
- If a different estimator (complementary, UKF) is selected, `estimator.c` routes `stateEstimator()` and the queue to that backend instead, keeping the rest of the firmware agnostic to which algorithm is active.

## Typical rates
| Stage | Nominal rate | Notes |
| --- | --- | --- |
| `sensorsWaitDataReady()` / IMU sampling | 1 kHz | Raw gyro/acc data, subsampled down to 100 Hz for prediction. |
| Stabilizer loop (`stateEstimator` + controllers) | 1 kHz | Determined by sensor ready interrupt. |
| EKF predict (`PREDICT_RATE`) | 100 Hz | Controlled by `PREDICTION_UPDATE_INTERVAL_MS` inside `kalmanTask()`. |
| Flow/TOF/baro/anchors decks | Sensor dependent | Updates are queued asynchronously; each file documents its gating. |
| Lighthouse sweeps | ~120 Hz per base station | Driven by sweep callbacks, gated by `outlierFilterLighthouse`. |
| LPS TDoA | Up to 350 Hz | Each packet immediately enqueues a measurement and triggers the outlier filter.

## Sensor → estimator → controller data path
1. **Physical sensor / deck driver**
   - Examples: `flowdeck_v1v2.c` produces `flowMeasurement_t`, `zranger*.c` produces `tofMeasurement_t`, `lpsTdoa3Tag.c` builds `tdoaMeasurement_t`, Lighthouse drivers enqueue `sweepAngleMeasurement_t` and `yawErrorMeasurement_t`, the barometer task injects `MeasurementTypeBarometer`.
2. **Measurement enqueue**
   - Drivers call `estimatorEnqueue*()` (declared in `src/modules/interface/estimator/estimator.h`) which copies the packet into a `measurement_t` union and pushes it to the estimator queue.
3. **Kalman task flush**
   - `updateQueuedMeasurements()` pulls the entries and calls the matching `kalmanCoreUpdateWith*()` function. Robust variants (`robustTwr`, `robustTdoa`) are selected via parameters.
4. **State export**
   - After `kalmanCoreFinalize()` and supervisor checks, `kalmanCoreExternalizeState()` fills `publishedEstimatorState`. The stabilizer loop reads it on its next 1 kHz iteration and feeds controllers, the commander, health monitors and logging.
5. **Logging/params**
   - `estimator_kalman.c` exposes EKF states, covariance diagonals and quaternion components via the `kalman` log group, and all tunable noises via the `kalman` parameter group (`pNAcc_xy`, `robustTdoa`, etc.).
