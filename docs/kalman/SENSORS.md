# Sensor Integration Guide

This document describes how each sensor type integrates into the Kalman filter estimator pipeline. For each sensor, we cover:

- **Hardware**: Physical sensor and mounting
- **Data source**: Driver and sampling rate
- **Measurement model**: How raw sensor data becomes a Kalman update
- **Gating and outliers**: Quality checks before accepting measurements
- **Tuning**: Measurement noise and runtime parameters

Related documents:
- [OVERVIEW.md](OVERVIEW.md) - Architecture and file inventory
- [PIPELINE.md](PIPELINE.md) - End-to-end data flow
- [STATE_AND_FRAMES.md](STATE_AND_FRAMES.md) - State vector and coordinate frames

---

## Quick Reference Table

| Sensor / Source | Measurement type (`MeasurementType*`) | Measurement model entry point | Notes on gating / timing |
| --- | --- | --- | --- |
| IMU (gyro/acc) | `MeasurementTypeGyroscope`, `MeasurementTypeAcceleration` | Sub-sampled in `estimator_kalman.c` via `axis3fSubSampler` → `kalmanCorePredict()` | Arrive at 1 kHz from `sensors.c`. The Kalman task averages them to 100 Hz for the predict step and adds process noise every loop. |
| Barometer | `MeasurementTypeBarometer` | `kalmanCoreUpdateWithBaro()` | Optional (`KALMAN_USE_BARO_UPDATE`). Maintains a reference height on the ground so the barometer does not drift when thrust is zero. |
| Flow deck (PMW3901) | `MeasurementTypeFlow` | `kalmanCoreUpdateWithFlow()` | Computes predicted pixel flow from velocity, altitude and gyro rates. Saturates altitude to avoid division by zero and logs predicted vs measured values in `kalman_pred`. |
| Z-ranger / ToF (VL53* pointing down) | `MeasurementTypeTOF` | `kalmanCoreUpdateWithTof()` | Projects the cone distance to a world-frame Z constraint using the current attitude. Only used when the sensor bore-sight is within ±15° of vertical. |
| Absolute height (VL53L1/VL6180 facing up or sideways) | `MeasurementTypeAbsoluteHeight` | `kalmanCoreUpdateWithAbsoluteHeight()` | Simple linear Jacobian on the Z state, useful for anchors mounted higher than the deck. |
| Loco Positioning System – Two Way Ranging | `MeasurementTypeDistance` | `kalmanCoreUpdateWithDistance()` or `kalmanCoreRobustUpdateWithDistance()` | Standard scalar update uses the line-of-sight Jacobian. Robust mode enables IRLS + G-M weighting (parameter `kalman.robustTwr`). |
| Loco Positioning System – Time Difference of Arrival | `MeasurementTypeTDOA` | `kalmanCoreUpdateWithTdoa()` or `kalmanCoreRobustUpdateWithTdoa()` | Computes the range-difference Jacobian. Standard mode uses an integrator-based outlier filter (`outlierFilterTdoaValidateIntegrator`). Robust mode adds IRLS weighting (parameter `kalman.robustTdoa`). Legacy step-based filtering can be enabled at compile time for troubleshooting. |
| External position (LPS absolute, mocap, scripted decks) | `MeasurementTypePosition` | `kalmanCoreUpdateWithPosition()` | Direct XYZ measurement; each axis is updated separately for efficiency. |
| External pose (Lighthouse, mocap) | `MeasurementTypePose` | `kalmanCoreUpdateWithPose()` | Fuses both position and quaternion information. Quaternion residuals are converted into attitude error states (`D0..D2`). |
| Lighthouse sweep angles | `MeasurementTypeSweepAngle` | `kalmanCoreUpdateWithSweepAngles()` | Converts sweep angles from the rotor frame to world-frame constraints, gated by `outlierFilterLighthouse`. Requires geometry from the Lighthouse deck (`rotorRot`, `sensorPos`, etc.). |
| Lighthouse yaw error | `MeasurementTypeYawError` | `kalmanCoreUpdateWithYawError()` | Injects a yaw correction derived from Lighthouse crossing-beams into the attitude error state `D2`. |
| Flow deck derived height | `MeasurementTypeFlow` (side channel) | `kalmanCoreUpdateWithFlow()` | Uses flow + range to compute velocity and implicitly constrains height through the Jacobian. |

**Implementation Notes**:
- `MeasurementTypeDistance` also has a `kalmanCoreRobustUpdateWithDistance()` path for outlier rejection.
- `MeasurementTypePose` updates both position (`stdDevPos`) and orientation (`stdDevQuat`).
- `measurement_t` definitions are in [src/modules/interface/estimator/estimator.h](../../src/modules/interface/estimator/estimator.h) and ultimately defined in [stabilizer_types.h](../../src/modules/interface/stabilizer_types.h).
- Every measurement handler uses `kalmanCoreScalarUpdate()` internally except for the robust variants, which pre-compute their own gain/covariance and call `kalmanCoreUpdateWithPKE()`.

---

## Table of Contents

1. [IMU (Accelerometer + Gyroscope)](#imu-accelerometer--gyroscope)
2. [Optical Flow Deck](#optical-flow-deck)
3. [Barometer](#barometer)
4. [Lighthouse Positioning System](#lighthouse-positioning-system)
5. [Loco Positioning System (UWB)](#loco-positioning-system-uwb)
6. [Motion Capture / External Pose](#motion-capture--external-pose)
7. [State Observability Summary](#state-observability-summary)
8. [Debugging Sensor Integration](#debugging-sensor-integration)

---

## IMU (Accelerometer + Gyroscope)

### Hardware

**Crazyflie 2.1**: BMI088 (16-bit, SPI)
- Accelerometer: ±3g, 1.5 kHz ODR
- Gyroscope: ±2000°/s, 1 kHz ODR

**Crazyflie 2.0**: MPU9250 (16-bit, I2C/SPI)
- Accelerometer: ±8g, 1 kHz ODR
- Gyroscope: ±2000°/s, 1 kHz ODR

**Mounting**: Aligned with body frame (no rotation required)

### Data Source

**Driver**: [src/drivers/src/bmi088_*.c](../../src/drivers/src/) or [src/drivers/src/mpu9250.c](../../src/drivers/src/mpu9250.c)

**Sampling**:
- Raw IMU data sampled at **1 kHz** by sensor driver
- Interrupt triggers `sensorsTask` to read data
- Data passed to `stabilizerTask` (1 kHz) via queue

**Pre-processing** (in `sensorsTask`):
1. Apply factory calibration (offset, scale)
2. Convert gyro from deg/s → rad/s
3. Optionally apply runtime bias correction
4. Package into `sensorData_t`

### Role in Kalman Filter

The IMU provides the **prediction step** inputs for the EKF:

**Gyroscope** (angular velocity ω):
- Used in `kalmanCorePredict()` to integrate quaternion: `q_new = q_old ⊗ exp(ω·dt/2)`
- Propagates attitude (rotation matrix `R`) forward in time

**Accelerometer** (specific force a):
- Measures `a_body = R^T · (a_true + g_world)` where g = [0, 0, -9.81] m/s²
- Used to propagate velocity: `v_new = v_old + R·(a - g_body)·dt`
- Provides gravity reference for roll/pitch observability

### Not a Measurement Update

**Important**: The IMU is NOT treated as a measurement in the update step. It drives the **prediction** (time propagation) only.

- No measurement Jacobian `H` for IMU
- No innovation or Kalman gain computed
- Process noise `Q` accounts for IMU errors (bias drift, noise)

### Process Noise Injection

After prediction, `kalmanCoreAddProcessNoise()` inflates the covariance to account for IMU uncertainty:

**Implementation** ([kalman_core.c](../../src/modules/src/kalman_core/kalman_core.c)):
```c
// Velocity process noise (accel uncertainty)
P[KC_STATE_PX][KC_STATE_PX] += params->procNoiseAcc_xy * dt²
P[KC_STATE_PY][KC_STATE_PY] += params->procNoiseAcc_xy * dt²
P[KC_STATE_PZ][KC_STATE_PZ] += params->procNoiseAcc_z * dt²

// Attitude process noise (gyro drift)
P[KC_STATE_D0][KC_STATE_D0] += params->procNoiseAtt * dt²
P[KC_STATE_D1][KC_STATE_D1] += params->procNoiseAtt * dt²
P[KC_STATE_D2][KC_STATE_D2] += params->procNoiseAtt * dt²
```

**Tuning Parameters** (`kalmanCoreParams_t`):
- `procNoiseAcc_xy`: Horizontal accel process noise (default: ~0.05 m/s²)
- `procNoiseAcc_z`: Vertical accel process noise (default: ~1.0 m/s²)
- `procNoiseVel`: Velocity process noise (drag model uncertainty)
- `procNoiseAtt`: Attitude process noise (gyro drift, ~1e-6 rad²/s)

### Timing

**Prediction rate**:
- Stabilizer task calls estimator at **1 kHz**
- Kalman task aggregates IMU samples and predicts at **250-500 Hz** (configurable)
- `kalmanCorePredict()` accumulates multiple IMU samples if needed

**Latency**:
- IMU → prediction: ~1-2 ms (sensor ISR → sensorsTask → stabilizerTask → kalmanTask)

### Coordinate Frames

- **Gyro output**: Angular velocity in **body frame** [rad/s]
- **Accel output**: Specific force in **body frame** [m/s² or g]
- **Quaternion integration**: World→Body rotation
- **Velocity integration**: Body-frame velocities `S[KC_STATE_PX/PY/PZ]`

See [STATE_AND_FRAMES.md - Coordinate Frames](STATE_AND_FRAMES.md#coordinate-frames) for detailed frame definitions.

---

## Optical Flow Deck

### Hardware

**Flow Deck 2.0**:
- **PMW3901 optical flow sensor**: Tracks pixel motion at ~400 Hz
- **VL53L1X Time-of-Flight (ToF) range sensor**: Measures height above ground (4 Hz)

**Mounting**: Bottom-facing, may be rotated 45° depending on deck version

### Data Source

**Driver**: [src/deck/drivers/src/flowdeck_v1v2.c](../../src/deck/drivers/src/flowdeck_v1v2.c)

**Sampling**:
- Flow sensor: ~400 Hz motion detection interrupt
- ToF sensor: ~4 Hz I2C polling
- Data enqueued as `flowMeasurement_t` and `tofMeasurement_t`

**Pre-processing** (in deck driver):
1. Read pixel displacement (dpixelX, dpixelY) from PMW3901
2. Read range (mm) from VL53L1X
3. Apply 45° rotation compensation if needed
4. Enqueue measurement packet to `measurementQueue`

### Measurement Models

#### Flow Measurement

**Model file**: [src/modules/src/kalman_core/mm_flow.c](../../src/modules/src/kalman_core/mm_flow.c)

**Measurement equation**:
```
Expected flow [rad/s] = (v_body / height) - ω_body
```

Where:
- `v_body`: Body-frame velocity `[S[KC_STATE_PX], S[KC_STATE_PY]]`
- `height`: Estimated altitude above ground (from ToF or Kalman Z estimate)
- `ω_body`: Angular velocity from gyro (de-rotation)

**Innovation**:
```c
z = measured_flow - expected_flow
```

**Jacobian** (`H`):
```c
∂h/∂S[KC_STATE_PX] = 1 / height
∂h/∂S[KC_STATE_PY] = 1 / height
∂h/∂S[KC_STATE_Z] = -v_body / height²
```

**Measurement noise** (`R`):
- Parameter: `kalman.flowStdFixed` (default: ~0.1 rad/s)
- Adaptive noise: Increases with low height or high velocity

**Gating**:
- Reject if `height < 0.1 m` (too close to ground)
- Reject if `|innovation| > 3σ` (outlier filter)
- Reject if gyro rate too high (>200°/s → unreliable flow)

#### ToF Measurement

**Model file**: [src/modules/src/kalman_core/mm_tof.c](../../src/modules/src/kalman_core/mm_tof.c)

**Measurement equation**:
```
Expected range = S[KC_STATE_Z] / cos(roll) / cos(pitch)
```

Where:
- `S[KC_STATE_Z]`: Estimated height in world frame
- Roll/pitch compensation: Sensor points downward in body frame, tilted from vertical

**Innovation**:
```c
z = measured_range - expected_range
```

**Jacobian** (`H`):
```c
∂h/∂S[KC_STATE_Z] = 1 / cos(roll) / cos(pitch)
∂h/∂S[KC_STATE_D0] = ... (attitude dependence)
∂h/∂S[KC_STATE_D1] = ... (attitude dependence)
```

**Measurement noise** (`R`):
- Parameter: `kalman.tofStdDev` (default: ~0.01 m at short range)
- Noise increases with distance (VL53L1X spec: ±5% at >1m)

**Gating**:
- Reject if `range > 2.0 m` (VL53L1X max reliable range)
- Reject if surface not Lambertian (low signal quality)
- Reject if `|innovation| > 3σ`

### Timing

**Flow update rate**: ~400 Hz (high rate)

**ToF update rate**: ~4 Hz (low rate)

**Measurement queue**: Both flow and ToF packets enqueued to `measurementQueue`, processed by `updateQueuedMeasurements()` in kalmanTask at 250-500 Hz.

### Coordinate Frames

- **Flow sensor**: Measures motion in **sensor X/Y plane** (bottom-facing)
- **Body frame conversion**: Apply 45° rotation if needed, then map to body +X/+Y
- **Height**: Assumed perpendicular to floor (world +Z if floor is level)

See [flowdeck_v1v2.c](../../src/deck/drivers/src/flowdeck_v1v2.c) for rotation matrix application.

### Tuning Parameters

| Parameter | Default | Units | Purpose |
|-----------|---------|-------|---------|
| `kalman.flowStdFixed` | 0.1 | rad/s | Flow measurement noise |
| `kalman.tofStdDev` | 0.01 | m | ToF measurement noise |
| `deck.flowMaxHeight` | 2.0 | m | Reject flow above this height |
| `deck.flowMinHeight` | 0.1 | m | Reject flow below this height |

---

## Barometer

### Hardware

**Crazyflie 2.1**: BMP388 (24-bit, I2C/SPI)
- Range: 300-1250 hPa
- Resolution: 0.016 Pa (~0.13 mm altitude)
- ODR: 1-200 Hz

**Crazyflie 2.0**: LPS25H (24-bit, I2C/SPI)
- Range: 260-1260 hPa
- Resolution: 0.01 hPa (~0.08 mm altitude)
- ODR: 1-25 Hz

### Data Source

**Driver**: [src/drivers/src/bmp3xx.c](../../src/drivers/src/bmp3xx.c) or [src/drivers/src/lps25h.c](../../src/drivers/src/lps25h.c)

**Sampling**:
- Barometer sampled at ~25 Hz by `baroTask` (FreeRTOS task)
- Converts pressure → altitude using standard atmosphere model:
  ```
  altitude = 44330 * (1 - (P/P0)^(1/5.255))
  ```
  where `P0 = 1013.25 hPa` (sea level reference)

**Pre-processing** (in `baroTask`):
1. Read raw pressure [Pa]
2. Apply temperature compensation
3. Convert to altitude [m] above sea level
4. **Reference height tracking**: Subtract ground-level pressure to get relative altitude
5. Enqueue as `baroMeasurement_t`

### Measurement Model

**Implementation**: Inline in [src/modules/src/estimator/estimator_kalman.c](../../src/modules/src/estimator/estimator_kalman.c)

**Measurement equation**:
```
Expected altitude = S[KC_STATE_Z]
```

**Innovation**:
```c
z = measured_altitude - S[KC_STATE_Z]
```

**Jacobian** (`H`):
```c
H = [0, 0, 1, 0, 0, 0, 0, 0, 0]  // Only updates Z position
```

**Measurement noise** (`R`):
- Parameter: `kalman.baroStdDev` (default: ~0.2 m)
- Accounts for atmospheric turbulence, sensor noise

**Gating**:
- Reject if `|innovation| > 5σ` (outlier threshold)
- Disable updates if `kalman.baroEnable = 0`

### Reference Height Tracking

**Problem**: Absolute barometric pressure drifts with weather (±3 hPa/hour = ±25 m/hour altitude error)

**Solution**: Track ground-level reference pressure `P_ref`:
1. At initialization, assume `Z = 0` → set `P_ref = P_measured`
2. Slowly adapt `P_ref` if drone is on ground (low velocity)
3. Measurement becomes: `altitude_relative = altitude_absolute - altitude_ref`

**Implementation**: See `baroTask` in [src/modules/src/sensors.c](../../src/modules/src/sensors.c)

### Timing

**Update rate**: ~25 Hz (slower than IMU, faster than UWB)

**Latency**: ~40 ms (I2C read + queue + Kalman task processing)

### Coordinate Frames

- **Measurement**: Altitude in **world +Z** direction (vertical up)
- **State**: `S[KC_STATE_Z]` is world-frame Z position

### Tuning Parameters

| Parameter | Default | Units | Purpose |
|-----------|---------|-------|---------|
| `kalman.baroStdDev` | 0.2 | m | Barometer measurement noise |
| `kalman.baroEnable` | 1 | bool | Enable/disable baro updates |
| `baro.aslTimeout` | 2000 | ms | Timeout for reference tracking |

### Use Cases

- **Indoor flight**: Provides Z-axis stabilization when no other height sensor available
- **Outdoor flight**: Complements GPS altitude (smoother short-term estimate)
- **Flow deck backup**: If ToF out of range (>2m), baro provides height reference

---

## Lighthouse Positioning System

### Hardware

**Lighthouse Deck** (HTC Vive / SteamVR Lighthouse compatible):
- **4 photodiodes** (TS4231 sensors) on deck PCB
- **2 base stations** (V1 or V2) emitting IR laser sweeps
- **Deck locations**: Each sensor has 3D position + normal vector in body frame

**Principle**:
1. Base stations emit horizontal and vertical laser sweeps
2. Each photodiode detects sweep timing → computes sweep angles (θ, φ)
3. Known base station geometry + sweep angles → 3D position triangulation

### Data Source

**Driver**: [src/deck/drivers/src/lighthouse.c](../../src/deck/drivers/src/lighthouse.c)

**Sampling**:
- Photodiode interrupts trigger angle measurement (~60 Hz per base station)
- Pulseprocessor ([pulseprocessor_v2.c](../../src/deck/drivers/src/pulseprocessor_v2.c)) decodes sweep angles
- Calibration data: `lighthouseCoreState.baseStationGeo[2]` (positions, orientations)
- Sensor positions: `lighthouseCalibration.sensorPositionsAndNormals[4]` (body frame)

**Pre-processing** (in lighthouse task):
1. Decode sweep timestamps → angles (radians)
2. Apply sensor calibration (timing offsets, lens distortion)
3. Enqueue `sweepAngleMeasurement_t` and `yawErrorMeasurement_t`

### Measurement Models

#### Sweep Angle Measurement

**Model file**: [src/modules/src/kalman_core/mm_sweep_angles.c](../../src/modules/src/kalman_core/mm_sweep_angles.c)

**Measurement equation** (simplified):
```
Expected sweep angle = project_3d_to_sweep_angle(sensor_position_world, basestation_geometry)
```

Where:
- `sensor_position_world = body_position + R * sensor_offset_body`
- `R`: Rotation matrix from `kalmanCoreData_t.R`
- `basestation_geometry`: Origin, rotation, sweep plane normal

**Innovation**:
```c
z = measured_angle - expected_angle
```

**Jacobian** (`H`):
- Complex 3D projection derivatives
- Depends on sensor position, base station geometry, sweep plane orientation
- See `lighthouseMeasurementModelSweepAnglesGetJacobian()` in [mm_sweep_angles.c](../../src/modules/src/kalman_core/mm_sweep_angles.c)

**Measurement noise** (`R`):
- Parameter: `kalman.lighthouseStdDev` (default: ~0.001 rad = 0.057°)
- Higher noise if signal quality low (weak pulse)

**Gating**:
- Reject if `|innovation| > 3σ`
- Reject if timestamp jitter too high (multipath interference)
- Outlier filter: `outlierFilterLighthouse*` in [src/utils/src/](../../src/utils/src/)

#### Yaw Error Measurement

**Model file**: [src/modules/src/kalman_core/mm_yaw_error.c](../../src/modules/src/kalman_core/mm_yaw_error.c)

**Measurement equation**:
```
Expected yaw = atan2(base_station_y, base_station_x)  // in world frame
```

**Innovation**:
```c
z = measured_yaw - expected_yaw
```

**Jacobian** (`H`):
```c
H[KC_STATE_D2] = 1  // Only updates yaw error state
```

**Measurement noise** (`R`):
- Parameter: `kalman.lighthouseYawStdDev` (default: ~0.01 rad)

**Purpose**: Provides yaw observability (otherwise yaw drifts with gyro integration alone)

### Calibration Requirements

**Base Station Calibration**:
1. **Geometry estimation**: Run calibration routine to estimate base station positions/orientations
2. **Storage**: Saved in `lighthouseCoreState.baseStationGeo[2]`
3. **Method**: Sweep deck in known pattern, solve for geometry (bundle adjustment)

**Sensor Calibration**:
1. **Factory calibration**: Sensor positions/normals in body frame stored in deck EEPROM
2. **Runtime**: Loaded into `lighthouseCalibration.sensorPositionsAndNormals[4]`

### Timing

**Update rate**: ~60 Hz per base station (120 Hz total with 2 base stations)

**Latency**: ~8-16 ms (photodiode interrupt → angle calculation → enqueue → Kalman update)

### Coordinate Frames

- **World frame**: Base station geometry defines world origin and orientation
- **Body frame**: Sensor offsets expressed in body frame
- **Sweep angles**: Measured in base station frame (horizontal/vertical planes)

**Transformation**:
```c
sensor_world = S[KC_STATE_X:Z] + R * sensor_body
expected_angle = project(sensor_world, basestation_geometry)
```

### Tuning Parameters

| Parameter | Default | Units | Purpose |
|-----------|---------|-------|---------|
| `kalman.lighthouseStdDev` | 0.001 | rad | Sweep angle measurement noise |
| `kalman.lighthouseYawStdDev` | 0.01 | rad | Yaw error measurement noise |
| `lighthouse.bsCalibrated` | 0/1 | bool | Base station geometry valid |
| `lighthouse.method` | 0 | enum | Positioning algorithm (0=crossing, 1=sweep) |

### Use Cases

- **Precision indoor positioning**: ~cm-level accuracy in 5×5×3m volume
- **Full 6-DOF pose**: Position + orientation (yaw) observability
- **Drift-free**: No integration drift (unlike optical flow)

### Known Limitations

- **Line of sight required**: At least 2 base stations must be visible
- **Reflections**: IR reflections from shiny surfaces cause outliers (mitigated by gating)
- **Calibration sensitive**: Base station geometry must be accurately calibrated

---

## Loco Positioning System (UWB)

### Hardware

**Loco Positioning Deck** (DWM1000 UWB transceiver):
- **Frequency**: 3.5-6.5 GHz ultra-wideband
- **Range**: 0-30 m typical
- **Accuracy**: ~10 cm (TDoA), ~5 cm (TWR)

**Infrastructure**:
- **Anchors**: 6-8 UWB nodes at known positions in world frame
- **Modes**:
  - **TDoA** (Time Difference of Arrival): Passive tag, no TX required
  - **TWR** (Two-Way Ranging): Active ranging, higher accuracy but limited tags

### Data Source

**Drivers**:
- TDoA: [src/deck/drivers/src/lpsTdoa*.c](../../src/deck/drivers/src/)
- TWR: [src/deck/drivers/src/lpsTwr*.c](../../src/deck/drivers/src/)

**Sampling**:
- **TDoA2/TDoA3**: ~20-50 Hz packet reception rate (anchor broadcasts received by tag)
- **TWR**: ~10-20 Hz ranging rate (limited by tag TX slots)

**Pre-processing** (in LPS task):
1. Receive UWB packet with timestamps
2. Compute TDoA (time difference) or range (distance)
3. Apply clock drift compensation (anchor/tag clock skew)
4. Lookup anchor positions from `locoPositioningState`
5. Enqueue `tdoaMeasurement_t` or `distanceMeasurement_t`

### Measurement Models

#### TDoA Measurement

**Model files**:
- [src/modules/src/kalman_core/mm_tdoa.c](../../src/modules/src/kalman_core/mm_tdoa.c)
- [src/modules/src/kalman_core/mm_tdoa_robust.c](../../src/modules/src/kalman_core/mm_tdoa_robust.c)

**Measurement equation**:
```
TDoA = (distance_to_anchor_A - distance_to_anchor_B) / speed_of_light
```

Where:
```c
distance_A = ||S[KC_STATE_X:Z] - anchor_A_position||
distance_B = ||S[KC_STATE_X:Z] - anchor_B_position||
```

**Innovation**:
```c
z = measured_tdoa - expected_tdoa
```

**Jacobian** (`H`):
```c
∂h/∂S[KC_STATE_X] = (x - x_A)/d_A - (x - x_B)/d_B
∂h/∂S[KC_STATE_Y] = (y - y_A)/d_A - (y - y_B)/d_B
∂h/∂S[KC_STATE_Z] = (z - z_A)/d_A - (z - z_B)/d_B
```

**Measurement noise** (`R`):
- Parameter: `kalman.tdoaStdDev` (default: ~0.15 m for TDoA2, ~0.10 m for TDoA3)
- Robust variant (`mm_tdoa_robust.c`): Huber loss for outlier rejection

**Gating**:
- Reject if anchor geometry degenerate (anchors collinear with tag)
- Reject if `|innovation| > 3σ`
- Robust variant: Downweight measurements with large residuals (M-estimation)

**Outlier Filtering**:
- Pre-filter: [src/utils/src/outlierFilterTdoa*.c](../../src/utils/src/)
- Histogram-based filtering: Reject measurements inconsistent with recent history
- Geometry check: Reject if anchor configuration is poor (high GDOP)

**TDoA2 vs TDoA3**:
- **TDoA2**: Original protocol, 20-30 Hz update rate
- **TDoA3**: Improved protocol with outlier filtering, 40-50 Hz update rate

#### TWR (Two-Way Ranging) Measurement

**Model files**:
- [src/modules/src/kalman_core/mm_distance.c](../../src/modules/src/kalman_core/mm_distance.c)
- [src/modules/src/kalman_core/mm_distance_robust.c](../../src/modules/src/kalman_core/mm_distance_robust.c)

**Measurement equation**:
```
Expected distance = ||S[KC_STATE_X:Z] - anchor_position||
```

**Innovation**:
```c
z = measured_distance - expected_distance
```

**Jacobian** (`H`):
```c
∂h/∂S[KC_STATE_X] = (x - x_anchor) / distance
∂h/∂S[KC_STATE_Y] = (y - y_anchor) / distance
∂h/∂S[KC_STATE_Z] = (z - z_anchor) / distance
```

**Measurement noise** (`R`):
- Parameter: `kalman.twrStdDev` (default: ~0.05 m)
- More accurate than TDoA (absolute range vs. range difference)

**Gating**:
- Reject if `distance > 30 m` (UWB max range)
- Reject if `|innovation| > 3σ`

### Anchor Calibration

**Manual configuration**:
```c
// In lpsTdoaTag.c or lpsTwrTag.c
anchorPosition[0] = {0.0, 0.0, 2.0};  // Anchor 0 at (0, 0, 2m)
anchorPosition[1] = {5.0, 0.0, 2.0};  // Anchor 1 at (5m, 0, 2m)
// ...
```

**Automatic calibration** (experimental):
- Use known tag positions to estimate anchor locations
- Not commonly used (manual survey more reliable)

### Timing

**TDoA update rate**: 20-50 Hz (depends on anchor count and traffic)

**TWR update rate**: 10-20 Hz (limited by tag TX scheduling)

**Latency**: ~20-40 ms (UWB packet RX → driver processing → enqueue → Kalman update)

### Coordinate Frames

- **Anchor positions**: Defined in **world frame** (user-configured)
- **Tag position**: `S[KC_STATE_X/Y/Z]` in **world frame**
- **Ranges/TDoAs**: Scalar distances (frame-independent)

**Important**: Anchor coordinate system defines the world frame origin and orientation. Ensure anchors are surveyed accurately.

### Tuning Parameters

| Parameter | Default | Units | Purpose |
|-----------|---------|-------|---------|
| `kalman.tdoaStdDev` | 0.15 | m | TDoA measurement noise (TDoA2) |
| `kalman.tdoa3StdDev` | 0.10 | m | TDoA measurement noise (TDoA3) |
| `kalman.twrStdDev` | 0.05 | m | TWR measurement noise |
| `kalman.robustTdoa` | 0/1 | bool | Use robust TDoA model |
| `kalman.robustTwr` | 0/1 | bool | Use robust TWR model |
| `lps.mode` | 0 | enum | 0=auto, 1=TWR, 2=TDoA2, 3=TDoA3 |

### Use Cases

- **Indoor navigation**: Room-scale positioning without external infrastructure (just anchors)
- **Multi-drone**: TDoA supports many drones (passive tags), TWR limited to ~10 drones
- **Accuracy vs throughput**: TWR more accurate but slower, TDoA faster but noisier

### Known Limitations

- **Multipath**: Indoor reflections cause ranging errors (mitigated by robust filters)
- **NLOS**: Non-line-of-sight anchors degrade accuracy (gating helps)
- **Anchor geometry**: Poor geometry (all anchors on ceiling) → poor Z accuracy

---

## Motion Capture / External Pose

### Hardware

**External Systems**:
- **Vicon / OptiTrack / Qualisys**: Infrared motion capture (sub-mm accuracy)
- **Custom localization**: Computer vision, SLAM, external tracking

**Interface**: CRTP (Crazyflie Real-Time Protocol) over radio or USB

### Data Source

**Service**: [src/modules/src/crtp_localization_service.c](../../src/modules/src/crtp_localization_service.c)

**Sampling**:
- External system sends pose packets via CRTP at ~10-100 Hz
- Packet types:
  - `POSITION`: (x, y, z) only
  - `POSE`: (x, y, z, qw, qx, qy, qz) full 6-DOF pose
  - `POSITION_PACKED`: Compressed position (multiple drones)

**Pre-processing** (in CRTP service):
1. Receive CRTP packet
2. Unpack position or pose
3. Convert from external frame to Crazyflie world frame (if coordinate transform configured)
4. Enqueue `positionMeasurement_t` or `poseMeasurement_t`

### Measurement Models

#### Position Measurement

**Model file**: [src/modules/src/kalman_core/mm_position.c](../../src/modules/src/kalman_core/mm_position.c)

**Measurement equation**:
```
Expected position = S[KC_STATE_X:Z]
```

**Innovation**:
```c
z_x = measured_x - S[KC_STATE_X]
z_y = measured_y - S[KC_STATE_Y]
z_z = measured_z - S[KC_STATE_Z]
```

**Jacobian** (`H`):
```c
H = [1, 0, 0, 0, 0, 0, 0, 0, 0]  // X position
H = [0, 1, 0, 0, 0, 0, 0, 0, 0]  // Y position
H = [0, 0, 1, 0, 0, 0, 0, 0, 0]  // Z position
```

**Measurement noise** (`R`):
- Parameter: `kalman.mocapStdDev` (default: ~0.01 m)
- Assumes high-quality MoCap system

**Gating**:
- Reject if `|innovation| > 3σ`
- Optional velocity gating: Reject if implied velocity > threshold

#### Pose Measurement

**Model file**: [src/modules/src/kalman_core/mm_pose.c](../../src/modules/src/kalman_core/mm_pose.c)

**Measurement equation** (position):
```
Expected position = S[KC_STATE_X:Z]
```

**Measurement equation** (orientation):
```
Expected quaternion = q (from kalmanCoreData_t.q)
Orientation error = measured_q ⊗ q^-1 → Rodrigues parameters
```

**Innovation**:
- Position: Same as position-only measurement
- Orientation: Attitude error → updates `S[KC_STATE_D0/D1/D2]`

**Jacobian** (`H`):
- Position: `[I3x3, 0, 0]` (identity for X/Y/Z, zero for velocities/attitudes)
- Orientation: `[0, 0, I3x3]` (identity for D0/D1/D2)

**Measurement noise** (`R`):
- Position: `kalman.mocapStdDev` (default: 0.01 m)
- Orientation: `kalman.mocapAttStdDev` (default: 0.01 rad)

**Gating**:
- Reject if `|innovation_pos| > 3σ_pos`
- Reject if `|innovation_att| > 3σ_att`

### Coordinate Frame Transformation

**Problem**: External system (e.g., Vicon) may use different coordinate convention (e.g., X=right, Y=forward, Z=up)

**Solution**: Apply transformation matrix before enqueuing:
```c
// Example: Vicon (RFU) → Crazyflie (FLU)
x_cf = y_vicon
y_cf = -x_vicon
z_cf = z_vicon
```

**Configuration**: Set via parameters or CRTP command

### Timing

**Update rate**: Depends on external system (typically 10-100 Hz)

**Latency**: ~10-50 ms (external system → radio → CRTP service → enqueue → Kalman update)

### Use Cases

- **Ground truth**: Compare Kalman estimate to MoCap for algorithm validation
- **High-precision control**: Use MoCap as primary position reference
- **Multi-drone swarms**: Centralized tracking system feeds positions to all drones

### Tuning Parameters

| Parameter | Default | Units | Purpose |
|-----------|---------|-------|---------|
| `kalman.mocapStdDev` | 0.01 | m | MoCap position measurement noise |
| `kalman.mocapAttStdDev` | 0.01 | rad | MoCap orientation measurement noise |
| `locSrv.extPosStdDev` | 0.01 | m | External position std dev (override) |
| `locSrv.extQuatStdDev` | 0.01 | rad | External quaternion std dev (override) |

---

## State Observability Summary

### State Observability by Sensor

| State | IMU | Flow | ToF | Baro | Lighthouse | UWB | MoCap |
|-------|-----|------|-----|------|------------|-----|-------|
| **X position** | ✗ (drift) | ✓ | ✗ | ✗ | ✓ | ✓ | ✓ |
| **Y position** | ✗ (drift) | ✓ | ✗ | ✗ | ✓ | ✓ | ✓ |
| **Z position** | ✗ (drift) | ✓ (weak) | ✓ | ✓ | ✓ | ✓ | ✓ |
| **X velocity** | ✗ (drift) | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ (via diff) |
| **Y velocity** | ✗ (drift) | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ (via diff) |
| **Z velocity** | ✗ (drift) | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ (via diff) |
| **Roll** | ✓ (gravity) | ✗ | ✓ (weak) | ✗ | ✗ | ✗ | ✓ |
| **Pitch** | ✓ (gravity) | ✗ | ✓ (weak) | ✗ | ✗ | ✗ | ✓ |
| **Yaw** | ✗ (drift) | ✗ | ✗ | ✗ | ✓ | ✗ | ✓ |

**Key**:
- ✓ = Observable (sensor provides direct or indirect measurement)
- ✗ = Not observable (sensor does not constrain this state)
- ✗ (drift) = Observable only via integration (unbounded error growth)
- ✓ (weak) = Weakly observable (coupled to other states)
- ✗ (via diff) = Can be differentiated from position, but not directly measured

### Sensor Fusion Strategies

**Minimal Configuration** (no external infrastructure):
- IMU + Flow deck → Position/velocity in body frame (drift-free horizontal, ToF-based height)
- IMU + Barometer → Altitude stabilization (drift-compensated with baro reference)

**Indoor Positioning** (infrastructure required):
- IMU + Lighthouse → Full 6-DOF pose (cm-level accuracy, yaw observable)
- IMU + UWB (TDoA) → Position only (10 cm accuracy, no yaw)
- IMU + UWB (TWR) → Position only (5 cm accuracy, no yaw)

**High-Precision** (lab environment):
- IMU + MoCap → Ground truth pose (mm-level, used for algorithm validation)

**Outdoor** (future):
- IMU + GPS → Position (m-level, yaw from magnetometer or motion)

---

## Debugging Sensor Integration

### Common Issues

**Problem**: "Position estimate diverges"
- **Check**: Which sensors are active? (`kalman.active*` logs)
- **Check**: Are measurement updates being accepted? (`kalman.rtUpdate`, `kalman.rtPred` logs)
- **Check**: Process noise too high? (`kalman.procNoiseAcc_*`, `kalman.procNoiseVel`)
- **Check**: Innovation gating too strict? (try increasing `*StdDev` parameters)

**Problem**: "Flow deck not working"
- **Check**: Height above ground (`range.zrange` log) – must be 0.1-2 m
- **Check**: Surface texture – plain surfaces (white floor) give poor flow
- **Check**: Gyro rate – high rotation (>200°/s) disables flow
- **Check**: `kalman.flowQuality` log – should be >0.5

**Problem**: "Lighthouse position jumps"
- **Check**: Base station geometry calibrated? (`lighthouse.bsCalibrated`)
- **Check**: Line of sight to 2+ base stations? (reflections cause outliers)
- **Check**: `lighthouse.rawAngle*` logs – should be smooth, not jumping

**Problem**: "UWB TDoA noisy"
- **Check**: Anchor geometry – need 3D coverage (not all on ceiling)
- **Check**: `kalman.tdoaNrOfAnchor` – should see 4+ anchors consistently
- **Check**: Use TDoA3 instead of TDoA2 (better outlier rejection)
- **Check**: Enable robust model: `kalman.robustTdoa = 1`

### Recommended Log Groups

**Estimator health**:
```
kalman.stateX/Y/Z      # Position estimate
kalman.statePX/PY/PZ   # Velocity estimate
kalman.varX/Y/Z        # Position variance (uncertainty)
kalman.varPX/PY/PZ     # Velocity variance
```

**Sensor status**:
```
kalman.rtUpdate        # Measurement update rate [Hz]
kalman.rtPred          # Prediction rate [Hz]
acc.x/y/z              # Accelerometer [g]
gyro.x/y/z             # Gyroscope [°/s]
baro.asl               # Barometer altitude [m]
range.zrange           # ToF range [m]
lighthouse.rawAngle*   # Lighthouse sweep angles
```

**Innovation monitoring**:
```
kalman.innov*          # Innovation (z) for each measurement type
```

Large innovations (>3σ) indicate:
- Measurement outlier (correctly rejected by gating)
- Model mismatch (measurement model incorrect)
- Divergence (estimator state far from truth)

---

## Further Reading

- **Kalman Core API**: [kalman_core.h](../../src/modules/interface/kalman_core/kalman_core.h)
- **Measurement Models**: [src/modules/src/kalman_core/mm_*.c](../../src/modules/src/kalman_core/)
- **Deck Drivers**: [src/deck/drivers/src/](../../src/deck/drivers/src/)
- **Estimator Interface**: [estimator.h](../../src/modules/interface/estimator/estimator.h)
- **Outlier Filters**: [src/utils/src/outlierFilter*.c](../../src/utils/src/)
- **Tuning Guide**: [OVERVIEW.md - Process and Measurement Noise](OVERVIEW.md#process-and-measurement-noise)
