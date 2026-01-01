# Kalman Filter and Estimator Subsystem Overview

This document provides a comprehensive overview of the Kalman filter and state estimator subsystem in the Crazyflie firmware. It serves as a navigable map for developers who need to understand, modify, or debug the state estimation pipeline.

## Quick Navigation

- **[PIPELINE.md](PIPELINE.md)**: Step-by-step flow from sensors → estimator → controller
- **[STATE_AND_FRAMES.md](STATE_AND_FRAMES.md)**: State vector definition, covariance, coordinate frames, and units
- **[SENSORS.md](SENSORS.md)**: How each sensor enters the estimator (timing, gating, measurement models)

## Architecture Summary

The Crazyflie state estimator subsystem is designed as a modular, measurement-driven Extended Kalman Filter (EKF). The architecture consists of:

1. **Sensor Sources**: Hardware drivers and deck drivers that produce raw sensor data
2. **Measurement Queue**: A shared FIFO queue (`estimator.c`) that decouples sensor production from estimator consumption
3. **Estimator Task**: A FreeRTOS task (`estimator_kalman.c`) that dequeues measurements and updates state
4. **Kalman Core**: The EKF implementation (`kalman_core.c`) providing predict and update primitives
5. **Measurement Models**: Modular measurement update functions (`mm_*.c`) that compute Jacobians and innovations
6. **Outlier Filters**: Pre-filters that reject bad measurements before they reach the EKF
7. **Supervisor**: Safety logic (`kalman_supervisor.c`) that bounds state estimates
8. **Stabilizer Integration**: The control loop (`stabilizer.c`) reads the estimated state for feedback control

### Estimator Selection

The firmware supports multiple state estimators (selectable via parameter `stabilizer.estimator`):
- **Kalman** (default, ID=1): Full EKF with support for all sensors
- **Complementary** (ID=2): Simple complementary filter (IMU + baro)
- **UKF** (ID=3): Unscented Kalman Filter (experimental)

This documentation focuses on the **Kalman estimator**, which is the most capable and widely used.

## Directory Guide

- `src/modules/interface/estimator/`: Public headers (`estimator.h`, `estimator_kalman.h`, etc.) that define the RTOS-agnostic API, measurement structs and enqueue helpers used by decks, drivers and the stabilizer loop.
- `src/modules/src/estimator/`: Front-end glue that implements the queue, estimator selection and RTOS tasks for each backend (`estimator.c`, `estimator_kalman.c`, `estimator_complementary.c`, ...).
- `src/modules/interface/kalman_core/`: EKF-facing headers that describe the core state vector, params and measurement model entry points.
- `src/modules/src/kalman_core/`: Implementation of the EKF math (predict/finalize/process noise) and per-sensor measurement models under `mm_*`.
- `src/modules/interface/outlierfilter/` & `src/modules/src/outlierfilter/`: Filtering helpers that gate measurements before they reach the EKF.
- `src/deck/drivers/src/`: Deck drivers that produce sensor measurements (Lighthouse, LPS, flow, range, etc.)
- `src/hal/src/`: Hardware abstraction layer for IMU, barometer, and other base sensors
- `docs/kalman/`: High-level documentation (this file, pipeline/state/sensor descriptions).

## File Inventory

The table below maps every source file that takes part in the Crazyflie Extended Kalman Filter (EKF) pipeline. Follow the "Path" links directly in the repository for deeper reading.

| Path | Category | Key functions / structs | Responsibility |
| --- | --- | --- | --- |
| `src/modules/src/stabilizer.c` | Stabilizer integration | `stabilizerInit`, `stabilizerTask`, `stateEstimatorGetType` | Owns the main control loop, selects the Kalman estimator through params, pulls EKF outputs each loop and forwards them to controllers/commanders. |
| `src/modules/[src,interface]/estimator/estimator.[c,h]` | Estimator interface | `measurement_t`, `stateEstimator*`, `estimatorEnqueue*`, `estimatorDequeue` | Defines the measurement_t struct; handles estimator API, including intial estimator selection, init, switch, and update; creates measurement_t queue and implements estimator-independent enqueue/dequeue functions that deck drivers use to feed sensor data into whichever estimator backend is active. |
| `src/modules/interface/estimator/estimator_kalman.h` | Estimator interface | `estimatorKalmanInit`, `estimatorKalman`, `estimatorKalmanGetEstimatedPos/Rot` | Public API used by the stabilizer to start, run and query the Kalman estimator. |
| `src/modules/src/estimator/estimator_kalman.c` | Core estimator task | `estimatorKalmanTaskInit`, `kalmanTask`, `updateQueuedMeasurements`, log/param groups | FreeRTOS task that drives the predict/update loop, dequeues measurements, schedules process noise, exposes params/logs and copies the EKF state to the stabilizer. |
| `src/modules/interface/kalman_core/kalman_core.h` | Kalman core API | `kalmanCoreData_t`, `kalmanCoreParams_t`, `kalmanCorePredict`, `kalmanCoreScalarUpdate` | Defines the EKF state vector, covariance container and all predict/update helpers used by measurement model sources. |
| `src/modules/src/kalman_core/kalman_core.c` | Core filter implementation | `kalmanCoreInit`, `kalmanCorePredict`, `kalmanCoreAddProcessNoise`, `kalmanCoreFinalize`, `kalmanCoreExternalizeState` | Implements the predict step, process noise integration, Joseph-form scalar update, quaternion finalization and state externalization. |
| `src/modules/interface/kalman_core/kalman_core_params_defaults.h` | Configuration defaults | `KALMAN_CORE_DEFAULT_PARAMS_INIT` macro | Single source of truth for default process/measurement noise and initialization values used by estimator_kalman.c and Python bindings. |
| `src/modules/interface/kalman_supervisor.h` | Supervisor interface | `kalmanSupervisorIsStateWithinBounds` | Declares the supervisor hook consumed by estimator_kalman.c. |
| `src/modules/src/kalman_supervisor.c` | Supervisor | `kalmanSupervisorIsStateWithinBounds`, param group | Checks if the EKF state diverges outside configured bounds and triggers resets; exposes guard rails as parameters. |
| `src/modules/[src,interface]/kalman_core/mm_distance.[c,h]` | Sensor update (LPS TWR) | `kalmanCoreUpdateWithDistance` | Scalar update that projects one anchor distance measurement into the global XYZ states. |
| `src/modules/[src,interface]/kalman_core/mm_distance_robust.[c,h]` | Sensor update (robust TWR) | `kalmanCoreRobustUpdateWithDistance` | Iteratively reweighted distance update that mitigates UWB outliers by re-scaling P and K before calling `kalmanCoreUpdateWithPKE`. |
| `src/modules/[src,interface]/kalman_core/mm_tdoa.[c,h]`| Sensor update (LPS TDoA) | `kalmanCoreUpdateWithTdoa` | Computes the range-difference Jacobian, runs optional outlier filtering and feeds the scalar update with TDoA innovations. |
| `src/modules/[src,interface]/kalman_core/mm_tdoa_robust.[c,h]` | Sensor update (robust TDoA) | `kalmanCoreRobustUpdateWithTdoa` | Implements G-M robust statistics for TDoA (P/K weighting, gating, optional TWR assistance) before delegating to `kalmanCoreUpdateWithPKE`. |
| `src/modules/[src,interface]/kalman_core/mm_position.[c,h]` | Sensor update (absolute XYZ) | `kalmanCoreUpdateWithPosition` | Straightforward absolute position measurement update (external positioning systems). |
| `src/modules/[src,interface]/kalman_core/mm_pose.[c,h]`| Sensor update (pose+quat) | `kalmanCoreUpdateWithPose` | Updates position and quaternion error states from external pose inputs (e.g., Lighthouse, MoCap). |
| `src/modules/[src,interface]/kalman_core/mm_flow.[c,h]`| Sensor update (optical flow) | `kalmanCoreUpdateWithFlow`, log group `kalman_pred` | Uses body velocities, altitude and gyro rates to predict pixel flow and performs two scalar updates (x/y) with gyro de-rotation. |
| `src/modules/[src,interface]/kalman_core/mm_tof.[c,h]` | Sensor update (time-of-flight range) | `kalmanCoreUpdateWithTof` | Projects one range-to-surface measurement (z-ranger) into the Z state with geometry-aware Jacobian. |
| `src/modules/[src,interface]/kalman_core/mm_absolute_height.[c,h]` | Sensor update (VL53/VL6180 height) | `kalmanCoreUpdateWithAbsoluteHeight` | Simple scalar update tying the world Z state to an absolute height sensor in the deck frame. |
| `src/modules/[src,interface]/kalman_core/mm_yaw_error.[c,h]` | Sensor update (yaw fusion) | `kalmanCoreUpdateWithYawError` | Uses external yaw references (e.g., Lighthouse sweep) by injecting yaw error directly into the attitude error state. |
| `src/modules/[src,interface]/kalman_core/mm_sweep_angles.[c,h]` | Sensor update (Lighthouse sweeps) | `kalmanCoreUpdateWithSweepAngles` | Processes lighthouse sweep pairs, runs lighthouse-specific outlier filter and updates yaw/position through geometric Jacobians. |
| `src/modules/[src,interface]/outlierfilter/outlierFilterTdoa.[c,h]` | Measurement gating | `outlierFilterTdoaValidateIntegrator`, `OutlierFilterTdoaState_t` | Adaptive integrator-based gate that opens/closes the TDoA measurement acceptance window depending on residual statistics. |
| `src/modules/[src,interface]/outlierfilter/outlierFilterTdoaSteps.[c,h]` | Measurement gating (legacy) | `outlierFilterTdoaValidateSteps` | Step/bucket-based fallback filter that gradually widens/narrows acceptable TDoA residuals when the integrator filter is unavailable. |
| `src/modules/[src,interface]/outlierfilter/outlierFilterLighthouse.[c,h]` | Measurement gating (Lighthouse) | `outlierFilterLighthouseValidateSweep` | Maintains a dynamic acceptance window for lighthouse sweep angle residuals to prevent corrupt packets from disturbing the EKF. |
| `src/modules/interface/stabilizer_types.h` | Core data types | `state_t`, `sensorData_t`, `control_t`, `setpoint_t`, `measurement_t` (union) | Defines all measurement types (TDoA, distance, flow, ToF, height, position, pose, sweep angles, yaw error, gyro, accel, baro), state estimate output, sensor inputs, control signals, and setpoints. |
| `src/hal/src/sensors.c` | Sensor HAL task | `sensorsTask`, `sensorsAcquire`, sensor calibration | High-level sensor task that reads IMU/baro at ~500 Hz, performs calibration, and enqueues measurements to estimator. |
| `src/hal/interface/sensors.h` | Sensor HAL interface | `sensorsManufacturingTest`, `sensorsAreCalibrated` | Public sensor interface for initialization, testing, and calibration status. |
| `src/hal/src/sensors_bmi088_bmp3xx.c` | IMU+Baro driver (CF2.1) | `sensorsBmi088Bmp3xxInit`, `sensorsBmi088Bmp3xxAcquire` | Primary sensor driver for Crazyflie 2.1: BMI088 (accel+gyro) + BMP3xx (baro). Produces `sensorData_t` with calibrated accel, gyro, baro readings. |
| `src/hal/src/sensors_mpu9250_lps25h.c` | IMU+Baro driver (CF2.0) | `sensorsMpu9250Lps25hInit`, `sensorsMpu9250Lps25hAcquire` | Legacy sensor driver for Crazyflie 2.0: MPU9250 (accel+gyro+mag) + LPS25H (baro). |
| `src/deck/drivers/src/lighthouse.c` | Lighthouse deck driver | `lighthouseDeckInit`, `lighthouseTask`, `lighthouseRecvPulseISR` | Lighthouse deck driver: receives IR timing pulses in ISR, decodes base station sync/sweep signals, manages base station geometry. |
| `src/modules/src/lighthouse/lighthouse_core.c` | Lighthouse core processing | `lighthouseCoreUpdateSystemType`, `lighthouseCorePropagateAngles` | Core lighthouse processing: identifies V1/V2 system type, converts timing data to sweep angles, validates geometry. |
| `src/modules/src/lighthouse/lighthouse_position_est.c` | Lighthouse position estimation | `lighthousePositionEstimatePoseCrossingBeams`, `lighthousePositionEstimatorEnqueueAnglesToEstimator` | Computes position/orientation from sweep angles using base station geometry, enqueues sweep angle measurements to Kalman filter. |
| `src/modules/interface/lighthouse/lighthouse_position_est.h` | Lighthouse position interface | `lighthousePositionEstimateGeometry`, calibration structs | Public interface for lighthouse position estimation and sensor geometry. |
| `src/modules/src/lighthouse/lighthouse_throttle.c` | Lighthouse throttling | `lighthouseThrottleMeasurement` | Limits Lighthouse measurement update rate to prevent estimator queue overflow (important for multi-base-station scenarios). |
| `src/modules/src/lighthouse/lighthouse_storage.c` | Lighthouse geometry storage | `lighthouseStoragePersistGeometryData`, `lighthouseStorageRetrieveGeometryData` | Stores/retrieves base station calibration data (position, rotation matrices) in persistent storage (flash or RAM). |
| `src/deck/drivers/src/locodeck.c` | LPS deck driver | `locodeckInit`, `uwbTask`, `uwbTxRxCallback` | Loco positioning deck driver: initializes DWM1000 UWB radio, manages TDoA/TWR mode selection, dispatches packets to tag implementations. |
| `src/deck/drivers/src/lpsTdoa2Tag.c` | TDoA2 tag implementation | `lpsTdoa2TagInit`, `lpsTdoa2TagProcessPacket`, stats | TDoA version 2 tag-side: processes anchor broadcast packets, computes TDoA constraints from timing data, runs outlier filter, enqueues to estimator. |
| `src/deck/drivers/src/lpsTdoa3Tag.c` | TDoA3 tag implementation | `lpsTdoa3TagInit`, `lpsTdoa3TagProcessPacket`, hybrid mode | TDoA version 3 tag-side: improved clock sync, hybrid mode (combines TDoA+TWR), better outlier handling. |
| `src/deck/drivers/src/lpsTwrTag.c` | TWR tag implementation | `lpsTwrTagInit`, `lpsTwrTagStats`, ranging logic | Two-Way Ranging tag-side: initiates ranging exchanges with anchors, computes distance from round-trip time, enqueues distance measurements. |
| `src/deck/drivers/interface/lpsTwrTag.h` | TWR tag interface | `lpsTwrTagSetRangingState` | Public interface for TWR tag configuration and control. |
| `src/deck/drivers/src/flowdeck_v1v2.c` | Flow deck driver | `flowdeck2Init`, `flowTask`, `pmw3901ReadMotion` | Optical flow deck driver: reads PMW3901 sensor at ~100 Hz, produces pixel velocity measurements (dx, dy, dt), includes ToF sensor (VL53L1X) for height. |
| `src/deck/drivers/src/zranger.c` | Z-ranger deck (VL53L0X) | `zrangerInit`, `zrangerTask`, `handleRangeUpdate` | Z-ranger deck driver: VL53L0X ToF sensor, measures ground distance (~4m range), enqueues ToF measurements for height estimation. |
| `src/deck/drivers/src/zranger2.c` | Z-ranger v2 deck (VL53L1X) | `zranger2Init`, `zranger2Task`, `handleRangeUpdate` | Z-ranger v2 deck driver: VL53L1X ToF sensor (improved range ~4m, faster rate ~50 Hz), enqueues ToF measurements. |
| `src/deck/drivers/src/multiranger.c` | Multi-ranger deck | `multirangerInit`, `multirangerTask`, 5-direction ToF | Multi-ranger deck: 5× VL53L1X ToF sensors (front, back, left, right, up), provides 360° obstacle sensing, enqueues multi-directional range data. |
| `src/deck/drivers/src/activeMarkerDeck.c` | Active marker deck | `activeMarkerDeckInit`, `activeMarkerDeckSetLed` | Active marker deck: provides switchable IR markers for motion capture systems (Qualisys, Vicon, OptiTrack), improves MoCap tracking. |
| `src/modules/src/crtp_localization_service.c` | CRTP localization service | `locSrvSendPacket`, external position/pose injection | CRTP localization service: receives external position/pose estimates via radio (from host PC running MoCap, simulator, etc.), enqueues to estimator. Supports position-only or full pose (position + quaternion). |
| `src/modules/src/range.c` | Range processing | `rangeEnqueueDownMeasurement`, `rangeGet`, `rangeGetDirection` | Range measurement aggregation: collects multi-directional ToF data from multi-ranger deck, provides unified interface for collision avoidance and logging. |
| `src/modules/interface/range.h` | Range interface | `rangeSet`, range direction enum | Public interface for range sensor data. |
| `src/utils/src/tdoa/tdoaEngine.c` | TDoA engine core | `tdoaEngineProcessPacket`, `tdoaEngineGetAnchorPosition`, clock sync | TDoA engine: manages anchor discovery, clock synchronization, computes TDoA measurements from packet timestamps, handles anchor geometry updates. |
| `src/utils/interface/tdoa/tdoaEngine.h` | TDoA engine interface | `tdoaEngineState_t`, `tdoaAnchorInfo_t` | TDoA engine data structures: anchor positions, receive timestamps, clock offsets. |
| `src/utils/src/tdoa/tdoaStats.c` | TDoA statistics | `tdoaStatsUpdate`, packet counters | TDoA statistics tracking: packet receive rates, anchor health, measurement quality metrics. |
| `src/modules/src/tdoaEngineInstance.c` | TDoA engine instance | `tdoaEngineInit`, singleton management | TDoA engine instance management: provides single shared TDoA engine for TDoA2/TDoA3 tag implementations. |
| `src/modules/src/collision_avoidance.c` | Collision avoidance | `collisionAvoidanceUpdateSetpoint`, `collisionAvoidanceIsActive` | Uses multi-ranger data to modify setpoints for obstacle avoidance (reduces velocity toward obstacles). Runs in stabilizer loop after estimator update. |
| `src/modules/src/supervisor.c` | High-level supervisor | `supervisorUpdate`, state machine (reset, ready, flying, landed, etc.) | High-level system supervisor: manages flight state machine, integrates Kalman supervisor checks, triggers emergency landings on estimator failures. |
| `src/modules/interface/supervisor.h` | Supervisor interface | `supervisorIsFlying`, `supervisorRequestCrashRecovery` | Public supervisor interface for flight state queries and recovery requests. |
| `src/modules/src/health.c` | Health monitoring | `healthShallWeRunTest`, health battery/estimator/propeller tests | System health monitoring: checks battery, estimator convergence, propeller health, reports warnings/errors. |
| `src/config/config.h` | System configuration | Feature flags, default values | System-wide configuration macros (not estimator-specific, but includes some estimator defaults). |

---

## Key Concepts

### Error-State Formulation

The Kalman core uses an **error-state EKF** formulation:
- **Nominal state**: Full state (position, velocity, quaternion) integrated forward in `kalmanCorePredict`
- **Error state**: 9D error vector (δp, δv, δθ) - position error, velocity error, attitude error - with covariance P (9×9)
- Updates correct the error state via measurement models, then the error is injected back into nominal state

**Advantages**:
- Quaternion normalization handled cleanly (errors are 3D, not 4D)
- Linearization errors minimized (errors remain small around zero)
- Covariance remains 9×9 instead of 10×10+ for full quaternion state

**State Vector** (`KC_STATE_DIM = 9`):
```
Error state δx = [δpx, δpy, δpz,    // position error (world frame, meters)
                  δvx, δvy, δvz,    // velocity error (world frame, m/s)
                  δθx, δθy, δθz]    // attitude error (axis-angle or small-angle, radians)
```

**Nominal state** (not in covariance):
```
Position: S (3D, meters)
Velocity: V (3D, m/s)
Quaternion: q (4D, unit norm, world → body rotation)
```

### Predict-Update Cycle

The EKF runs in two phases:

**1. Predict** (IMU-driven, ~500 Hz in `kalmanCorePredict`):
   - Integrate accel/gyro to update nominal position, velocity, quaternion
   - Propagate covariance using linearized continuous-time dynamics discretized to dt:
     - Build system matrix `A` (function of current attitude, acceleration, gyro rates)
     - Discretize: `F ≈ I + A*dt`, `G_disc ≈ G*dt`
     - Propagate: `P = F*P*F^T + G*Q*G^T`
   - Process noise `Q` represents IMU noise (accel noise → velocity/position uncertainty growth, gyro noise → attitude drift)

**2. Update** (asynchronous, measurement-driven, various measurement models):
   - Compute innovation: `y = z - h(x_nominal)` where `h` is the measurement prediction function
   - Compute Jacobian: `H = ∂h/∂δx` (derivative w.r.t. error state)
   - Compute innovation covariance: `S = H*P*H^T + R`
   - Compute Kalman gain: `K = P*H^T * S^-1`
   - Update error state: `δx = K*y`
   - Update covariance (Joseph form for numerical stability): `P = (I - K*H)*P*(I - K*H)^T + K*R*K^T`
   - Simplified form often used: `P = (I - K*H)*P`
   - Inject error into nominal state: position += δp, velocity += δv, accumulate attitude error δθ

**3. Finalize** (after all updates in a cycle, `kalmanCoreFinalize`):
   - Convert accumulated attitude error δθ to quaternion δq
   - Apply to nominal quaternion: `q = δq ⊗ q` (quaternion multiplication)
   - Renormalize quaternion: `q = q / ||q||`
   - Reset attitude error to zero: δθ = [0, 0, 0]

### Measurement Gating and Outlier Rejection

Most measurement models include **gating** to reject spurious measurements:

1. **Innovation-based threshold**: Reject if `|y| > threshold` (e.g., 0.5m for position, 3.0m for TDoA)
2. **Mahalanobis distance (chi-square test)**: Reject if `y^T * S^-1 * y > chi2_threshold`
   - Accounts for uncertainty (covariance P and measurement noise R)
   - Common threshold: χ² at 95% confidence (e.g., 3.84 for 1-DOF, 5.99 for 2-DOF)
3. **Robust estimation (M-estimation)**: Use Huber loss, Tukey biweight, or IRLS to downweight outliers
   - Instead of hard rejection, scale Kalman gain: `K_robust = w * K`
   - Implemented in `mm_tdoa_robust.c`, `mm_distance_robust.c`
4. **Outlier filters** (pre-filters before measurement model):
   - `outlierFilterTdoa.c`: Integrator-based adaptive gating
   - `outlierFilterLighthouse.c`: Dynamic window based on recent residuals

### Coordinate Frames

Understanding coordinate frames is critical for debugging measurement models.

**World Frame**:
- **Default**: Right-handed ENU (East-North-Up) - East is +X, North is +Y, Up is +Z
- Gravity: `g_world = [0, 0, -9.81]` m/s² (points down in ENU)
- Position `S` and velocity `V` estimates are in world frame
- Inertial frame (fixed during flight)

**Body Frame** (FRD - Front-Right-Down):
- Front (forward) is +X, Right is +Y, Down is +Z
- IMU measurements (accel, gyro) are in body frame
- Quaternion `q` encodes rotation: `v_body = q ⊗ v_world ⊗ q*` (world → body)
- Inverse rotation: `v_world = q* ⊗ v_body ⊗ q` (body → world)

**Sensor Frames** (deck-dependent):
- **Flow deck**: Downward-facing, X/Y aligned with body or rotated 45°
- **Lighthouse sensors**: 4 photodiodes on deck, each with 3D position in body frame
- **UWB antenna**: Single point, typically centered on deck

**Frame Transformations**:
- Rotation matrices: `R_world_to_body = quat_to_matrix(q)`
- Accel to world frame: `a_world = R_body_to_world * a_body - g_world`
- Gyro integration: `q_dot = 0.5 * q ⊗ [0, ωx, ωy, ωz]`

See [STATE_AND_FRAMES.md](STATE_AND_FRAMES.md) for complete frame definitions and transformations.

### Process Noise Tuning

Process noise `Q` models uncertainty growth during prediction. Key parameters (tunable via CRTP params):

| Parameter | Default | Description | Effect of Increasing |
|-----------|---------|-------------|----------------------|
| `kalman.pNAcc_xy` | ~0.5 m/s² | XY acceleration noise | Faster horizontal position/velocity adaptation to measurements |
| `kalman.pNAcc_z` | ~1.0 m/s² | Z acceleration noise | Faster vertical position/velocity adaptation |
| `kalman.pNVel` | 0 (often) | Velocity random walk | Allows velocity to drift (usually kept low) |
| `kalman.pNAtt` | ~0.01 rad/s | Attitude (gyro) noise | Faster attitude adaptation, compensates gyro drift |

**Tuning Guidelines**:
- **Too low**: Filter is overconfident, ignores measurements, slow to adapt to changes
- **Too high**: Filter is jittery, tracks measurement noise, less smooth
- **Flying vs grounded**: Process noise often increased when `quadIsFlying=1` to handle unmodeled aerodynamics

### Measurement Noise Tuning

Measurement noise `R` models sensor uncertainty. Each measurement model has its own R (sometimes tunable).

**Examples**:
- **TDoA**: `stdDev ~ 0.15m` (depends on anchor geometry, multipath)
- **TWR distance**: `stdDev ~ 0.10m` (more accurate than TDoA)
- **Optical flow**: `stdDev ~ 0.5 px/s` (depends on height, surface texture)
- **ToF range**: `stdDev ~ 0.02m` (VL53 sensors are quite accurate)
- **External position (MoCap)**: `locSrv.extPosStdDev`, default ~0.01m
- **Lighthouse sweep angles**: `stdDev ~ 0.001 rad` (very precise)

**Tuning Guidelines**:
- **Too low**: Measurements overweighted, filter jumps to noisy measurements
- **Too high**: Measurements underweighted, filter ignores good data
- Use actual sensor datasheets and empirical testing to set R

---

## Common Debugging Questions

| Question | Answer |
|----------|--------|
| **Where is the predict step?** | [kalman_core.c:kalmanCorePredict](../../src/modules/src/kalman_core/kalman_core.c) - called from estimator_kalman.c on every IMU update (~500 Hz) |
| **How does Lighthouse enter the filter?** | [lighthouse_position_est.c](../../src/modules/src/lighthouse/lighthouse_position_est.c) processes angles → [mm_sweep_angles.c](../../src/modules/src/kalman_core/mm_sweep_angles.c) performs update |
| **How does LPS TDoA enter the filter?** | [lpsTdoa3Tag.c](../../src/deck/drivers/src/lpsTdoa3Tag.c) computes TDoA constraints → [mm_tdoa.c](../../src/modules/src/kalman_core/mm_tdoa.c) or [mm_tdoa_robust.c](../../src/modules/src/kalman_core/mm_tdoa_robust.c) |
| **How does optical flow enter the filter?** | [flowdeck_v1v2.c](../../src/deck/drivers/src/flowdeck_v1v2.c) reads PMW3901 → [mm_flow.c](../../src/modules/src/kalman_core/mm_flow.c) |
| **How does external position (MoCap) enter?** | [crtp_localization_service.c](../../src/modules/src/crtp_localization_service.c) receives CRTP packets → [mm_position.c](../../src/modules/src/kalman_core/mm_position.c) or [mm_pose.c](../../src/modules/src/kalman_core/mm_pose.c) |
| **Where are process noise values set?** | Defaults in [kalman_core_params_defaults.h](../../src/modules/interface/kalman_core/kalman_core_params_defaults.h), tunable via params `kalman.pNAcc_xy`, `kalman.pNAcc_z`, `kalman.pNVel`, `kalman.pNAtt` |
| **Where is gating/outlier rejection done?** | Within each measurement model (e.g., `mm_tdoa.c`, `mm_flow.c` check innovation norms) AND in pre-filters (`outlierFilterTdoa.c`, `outlierFilterLighthouse.c`) |
| **What frames do setpoints vs state use?** | Both use **world frame (ENU)** for position/velocity; attitude is quaternion (world→body rotation) |
| **How to switch estimators?** | Set parameter `stabilizer.estimator` (1=Kalman, 2=Complementary, 3=UKF) via CRTP or cfclient |
| **How to reset Kalman state?** | Set parameter `kalman.resetEstimation = 1` - triggers re-initialization of position, velocity, covariance |
| **Why is my position estimate drifting?** | Check: (1) IMU calibration (`sensorsAreCalibrated`), (2) process noise too high, (3) no absolute position measurements (need Lighthouse, LPS, or MoCap), (4) measurement gating rejecting good data |
| **Why are measurements being rejected?** | Check logs: measurement model often logs gate status; verify measurement noise R and innovation thresholds; check outlier filter state |
| **How do I log Kalman state?** | Log variables: `kalman.stateX/Y/Z` (position), `kalman.vx/vy/vz` (velocity), `kalman.q0/q1/q2/q3` (quaternion), `kalman.statePX/PY/PZ` (covariance diagonals) |
| **Where is covariance initialized?** | `kalmanCoreInit` in [kalman_core.c](../../src/modules/src/kalman_core/kalman_core.c) - sets initial P diagonal from `kalmanCoreParams_t` |
| **What is `quadIsFlying` and why does it matter?** | Boolean flag indicating flight state; when true, process noise is often increased to handle unmodeled aerodynamic forces; set by supervisor or manually via param |
| **Where is the stabilizer loop?** | [stabilizer.c:stabilizerTask](../../src/modules/src/stabilizer.c) - runs ~500 Hz, calls `sensorsAcquire` → estimator update → controller → motors |

---

## Runtime Parameters and Logging

The Kalman estimator exposes many runtime parameters (read/write via CRTP) and log variables (read-only telemetry).

### Key Parameters

**Estimator Control**:
- `stabilizer.estimator` (uint8): Select estimator (1=Kalman, 2=Complementary, 3=UKF)
- `kalman.resetEstimation` (uint8): Write 1 to reset Kalman state and covariance
- `kalman.quadIsFlying` (uint8): Flight state flag (0=grounded, 1=flying) - affects process noise

**Process Noise** (float, units vary):
- `kalman.pNAcc_xy` (m/s²): Horizontal acceleration process noise
- `kalman.pNAcc_z` (m/s²): Vertical acceleration process noise
- `kalman.pNVel` (m/s): Velocity random walk (usually 0)
- `kalman.pNAtt` (rad/s): Attitude (gyro) process noise

**Measurement Model Selection**:
- `kalman.robustTdoa` (uint8): Enable robust TDoA (M-estimation)
- `kalman.robustTwr` (uint8): Enable robust TWR (M-estimation)

**External Localization** (CRTP service):
- `locSrv.extPosStdDev` (float, meters): Measurement noise for external position
- `locSrv.extQuatStdDev` (float, radians): Measurement noise for external quaternion

**Supervisor Bounds** (float, units vary):
- `kalSup.maxPos` (meters): Max position magnitude before triggering supervisor alert
- `kalSup.maxVel` (m/s): Max velocity magnitude before triggering supervisor alert

### Key Log Variables

**State Estimate**:
- `kalman.stateX/Y/Z` (float, meters): Position in world frame
- `kalman.vx/vy/vz` (float, m/s): Velocity in world frame (body frame in some firmware versions - check code!)
- `kalman.q0/q1/q2/q3` (float): Quaternion (q0=w, q1=x, q2=y, q3=z)

**Covariance (Uncertainty)**:
- `kalman.statePX/PY/PZ` (float, m²): Position variance (diagonal of P)
- `kalman.statePVX/PVY/PVZ` (float, (m/s)²): Velocity variance
- `kalman.statePA0/PA1/PA2` (float, rad²): Attitude variance

**Measurement Model Debug**:
- `kalman_pred.predNX/NY` (float, px/s): Optical flow prediction (from body velocity + gyro)
- `kalman_pred.measNX/NY` (float, px/s): Optical flow measurement (from PMW3901)
- `lighthouse.rawAngle0x/0y/1x/1y` (float, radians): Lighthouse sweep angles

**Supervisor**:
- `supervisor.info` (uint16, bitfield): Supervisor state flags (flying, tumbled, etc.)

**TDoA/TWR Debug** (varies by tag implementation):
- `tdoa.x/y/z` (float, meters): TDoA-based position estimate (independent of Kalman, for comparison)
- `tdoa.anchorCount` (uint8): Number of active anchors

---

## Configuration (Build-Time)

Build-time configuration via Kconfig (`src/modules/src/Kconfig`):

**Estimator Selection** (at least one must be enabled):
- `CONFIG_ESTIMATOR_KALMAN`: Enable Kalman filter (default=y)
- `CONFIG_ESTIMATOR_COMPLEMENTARY`: Enable complementary filter
- `CONFIG_ESTIMATOR_UKF`: Enable UKF

**Deck Support** (affects which measurement models are compiled):
- `CONFIG_DECK_LIGHTHOUSE`: Enable Lighthouse deck and mm_sweep_angles
- `CONFIG_DECK_LOCO`: Enable Loco Positioning (UWB) deck and mm_tdoa/mm_distance
- `CONFIG_DECK_FLOW`: Enable optical flow deck and mm_flow

**Outlier Filters**:
- Compiled based on deck support (e.g., Lighthouse outlier filter if `CONFIG_DECK_LIGHTHOUSE=y`)

**Platform-Specific**:
- `CONFIG_SENSORS_BMI088_BMP3XX`: Use BMI088+BMP3xx sensors (Crazyflie 2.1)
- `CONFIG_SENSORS_MPU9250_LPS25H`: Use MPU9250+LPS25H sensors (Crazyflie 2.0)

---

## Testing

The Kalman subsystem includes unit tests (C) and integration tests (Python).

**Unit Tests** (run on host with `make unit`):
- `test/modules/src/test_kalman_supervisor.c`: Kalman supervisor bounds checking
- `test/modules/src/kalman_core/test_mm_*.c`: Measurement model unit tests (absolute height, TDoA, etc.)
- `test/modules/src/kalman_core/kalman_core_mm_test_helpers.c`: Test helpers for measurement models

**Integration Tests** (Python bindings):
- `test_python/test_kalman_core.py`: End-to-end Kalman filter tests using Python bindings
- `bindings/util/estimator_kalman_emulator.py`: Offline Kalman emulator for log replay

**Test Coverage**:
- Core EKF math (predict, update, finalize)
- Measurement models (Jacobians, innovation computation)
- Supervisor logic (bounds checking, state machine)
- Outlier filters (gating thresholds)

**Running Tests**:
```bash
make unit                          # Run all C unit tests
python3 test_python/test_kalman_core.py  # Run Python integration tests
```

---

## Python Bindings and Offline Tools

The firmware provides Python bindings (SWIG) for offline development and testing.

**Bindings**:
- `bindings/cffirmware.i`: SWIG interface definition
- `bindings/setup.py`: Python package setup (`pip install -e bindings/`)

**Utilities**:
- `bindings/util/estimator_kalman_emulator.py`: Kalman filter emulator
  - Replays sensor logs through Kalman filter
  - Useful for parameter tuning without hardware
- `bindings/util/sd_card_file_runner.py`: Runs logged sensor data from SD card

**Use Cases**:
- **Parameter tuning**: Replay flight logs with different Q, R values
- **Debugging**: Step through estimator updates in Python debugger
- **Validation**: Compare firmware output to Python emulator
- **Algorithm development**: Prototype new measurement models in Python before porting to C

---

## Next Steps

- **Understand the pipeline**: Read [PIPELINE.md](PIPELINE.md) for sensor-to-control data flow
- **Learn the state vector**: Read [STATE_AND_FRAMES.md](STATE_AND_FRAMES.md) for state layout and coordinate systems
- **Dive into sensors**: Read [SENSORS.md](SENSORS.md) for sensor-specific integration details
- **Explore source code**: Use the inventory table above to navigate to specific files
- **Run tests**: `make unit` and review test cases for measurement models
- **Tune parameters**: Experiment with process/measurement noise using cfclient or Python bindings

---

## Contributing

When modifying the Kalman estimator subsystem:

1. **Preserve behavior**: Do not change numeric constants or logic unless fixing a bug
2. **Document changes**: Add/update comments for any new measurement models or parameters
3. **Test thoroughly**: Run unit tests (`make unit`) and flight tests before committing
4. **Update documentation**: Keep OVERVIEW.md, PIPELINE.md, STATE_AND_FRAMES.md, SENSORS.md in sync with code
5. **Follow conventions**:
   - Measurement models: `mm_<sensor>.c` with `kalmanCoreUpdateWith<Sensor>` function
   - Log groups: `kalman.*`, `kalman_pred.*`, `<sensor>.*`
   - Parameters: `kalman.*`, `kalSup.*`, `locSrv.*`

---

**Document Version**: 1.0
**Last Updated**: 2025-12-31
**Maintainers**: Crazyflie firmware contributors

For questions or issues with this documentation, please open an issue on the [crazyflie-firmware GitHub repository](https://github.com/bitcraze/crazyflie-firmware).
