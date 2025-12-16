# Estimator Pipeline

## Call graph
- `stabilizerTask()` (1 kHz sensor ticks) waits for `sensorsWaitDataReady()` and calls `stateEstimator()` once per loop.
- `stateEstimator()` dispatches to the active backend (`estimatorKalman()` when the Kalman estimator is selected via the `stabilizer.estimator` parameter).
- `estimatorKalman()` copies the latest EKF state into the caller-provided `state_t` and `xSemaphoreGive()`s `runTaskSemaphore` to wake the Kalman task.
- `kalmanTask()` (FreeRTOS task created by `estimatorKalmanTaskInit()`) performs predict, process-noise, measurement flush, finalize, supervisor guard and externalization.
- `kalmanCore*()` functions do the heavy math: `kalmanCorePredict()` (100 Hz), `kalmanCoreAddProcessNoise()` (every loop), `kalmanCoreUpdateWith*()` per measurement, and `kalmanCoreFinalize()` / `kalmanCoreExternalizeState()` before handing the results back to the stabilizer loop.

```
stabilizerTaskLoop()
  └─ stateEstimator(state_t*, stabilizerStep)
       └─ estimatorKalman(state, step)   # if Kalman estimator is selected
            ├─ copy latest state exported by kalmanTask
            └─ xSemaphoreGive(runTaskSemaphore) → wakes kalmanTask()
kalmanTask()  # FreeRTOS task created in estimatorKalmanTaskInit()
  ├─ axis3fSubSamplerAccumulate() via MeasurementType{Gyro,Acc} packets
  ├─ kalmanCorePredict() at 100 Hz (PREDICT_RATE)
  ├─ kalmanCoreAddProcessNoise() each loop
  ├─ updateQueuedMeasurements()  # drains queue and dispatches measurement models
  ├─ kalmanCoreFinalize()        # moves attitude error, rebuilds rotation matrix
  └─ kalmanCoreExternalizeState() copies state into taskEstimatorState
```

## Loop phases inside `kalmanTask()`
1. **IMU aggregation** – `axis3fSubSamplerAccumulate()`/`gyro` run at 1 kHz whenever `MeasurementTypeAcceleration/Gyroscope` packets arrive. When it is time to predict (every 10 ms) `axis3fSubSamplerFinalize()` produces an averaged, gravity-compensated sample.
2. **Predict step** – `kalmanCorePredict()` integrates the IMU sample through the continuous-time quadrotor model and updates the covariance (`PREDICT_RATE` = 100 Hz). A rate supervisor warns if the task drifts from that cadence.
3. **Process noise** – `kalmanCoreAddProcessNoise()` is called every loop (regardless of whether a predict step happened) so that low-pass filtered sensor fusion still inflates `P` according to real elapsed time.
4. **Measurement queue flush** – `updateQueuedMeasurements()` consumes every pending `measurement_t` dequeued via `estimatorDequeue()` and dispatches to the appropriate `kalmanCoreUpdateWith*()` function. Each measurement model gates or discards samples as needed (TDoA/lighthouse outlier filters, robust updates, etc.).
5. **Finalize** – `kalmanCoreFinalize()` moves the attitude-error states into the quaternion and recomputes the rotation matrix. If no measurement or prediction touched the state, this step is skipped to save work.
6. **Supervisor** – `kalmanSupervisorIsStateWithinBounds()` checks that position/velocity stay inside safe envelopes; violations set `resetEstimation` so the loop re-runs `estimatorKalmanInit()` on the next tick.
7. **Externalize** – The internal state is converted to the public `state_t` via `kalmanCoreExternalizeState()` while holding `dataMutex`, after which the stabilizer loop uses the new position, velocity, acceleration and attitude estimates.

## Measurement queue and threading
- Sensor drivers and deck apps call `estimatorEnqueue*()` helpers (or `estimatorEnqueue()` directly) from ISRs or worker tasks. Each helper fills a `measurement_t` union containing the measurement payload and posts it to the estimator queue.
- `stateEstimator()` (running in the stabilizer task) never blocks on sensors; it simply copies the latest state and wakes the Kalman task. All heavy lifting happens inside `kalmanTask()` where there is no control-loop latency pressure.
- Measurements are not accumulated per-type; every entry is consumed on the next `kalmanTask()` iteration to minimize latency between sensors and the EKF.

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
   - After `kalmanCoreFinalize()` and supervisor checks, `kalmanCoreExternalizeState()` fills `taskEstimatorState`. The stabilizer loop reads it on its next 1 kHz iteration and feeds controllers, the commander, health monitors and logging.
5. **Logging/params**
   - `estimator_kalman.c` exposes EKF states, covariance diagonals and quaternion components via the `kalman` log group, and all tunable noises via the `kalman` parameter group (`pNAcc_xy`, `robustTdoa`, etc.).
