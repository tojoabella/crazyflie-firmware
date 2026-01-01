# State Vector, Covariance, and Coordinate Frames

This document provides a detailed description of the Kalman filter state vector, covariance matrix, and coordinate frame conventions used throughout the Crazyflie firmware.

## Table of Contents

1. [State Vector Definition](#state-vector-definition)
2. [Coordinate Frames](#coordinate-frames)
3. [Covariance Matrix](#covariance-matrix)
4. [Externalized State](#externalized-state)
5. [Frame Transformations](#frame-transformations)
6. [Units and Conventions](#units-and-conventions)

---

## State Vector Definition

The Kalman filter uses a **9-dimensional error-state representation**. The state vector `S` stored in `kalmanCoreData_t.S[]` contains:

### State Vector Ordering (`kalmanCoreStateIdx_t`)

| Index | Symbol | Name | Meaning | Units | Frame |
|-------|--------|------|---------|-------|-------|
| **0** | `KC_STATE_X` | x | Position X (East or Forward) | meters [m] | World/Global |
| **1** | `KC_STATE_Y` | y | Position Y (North or Left) | meters [m] | World/Global |
| **2** | `KC_STATE_Z` | z | Position Z (Up) | meters [m] | World/Global |
| **3** | `KC_STATE_PX` | v<sub>x</sub> | Velocity along body +X (forward) | m/s | Body |
| **4** | `KC_STATE_PY` | v<sub>y</sub> | Velocity along body +Y (left) | m/s | Body |
| **5** | `KC_STATE_PZ` | v<sub>z</sub> | Velocity along body +Z (up) | m/s | Body |
| **6** | `KC_STATE_D0` | δθ<sub>x</sub> | Attitude error about body X axis | radians [rad] | Body (Rodrigues) |
| **7** | `KC_STATE_D1` | δθ<sub>y</sub> | Attitude error about body Y axis | radians [rad] | Body (Rodrigues) |
| **8** | `KC_STATE_D2` | δθ<sub>z</sub> | Attitude error about body Z axis | radians [rad] | Body (Rodrigues) |

**Total State Dimension**: `KC_STATE_DIM = 9`

### Additional State Representations

Beyond the 9D error state vector, `kalmanCoreData_t` maintains additional attitude representations:

| Member | Type | Purpose | Notes |
|--------|------|---------|-------|
| `q[4]` | float[4] | **Nominal quaternion** [w, x, y, z] | Unit quaternion representing world→body rotation; integrated during predict, normalized during finalize |
| `R[3][3]` | float[3][3] | **Rotation matrix** | Derived from quaternion during finalize; used for frame transformations (body→world: `R^T`, world→body: `R`) |
| `P[9][9]` | float[9][9] | **Covariance matrix** | Uncertainty of the 9D error state; symmetric positive semi-definite |

### Error-State vs. Nominal State

The Kalman filter uses an **error-state formulation** (also called indirect or delta formulation):

**Nominal State** (integrated forward, no covariance):
```
Position:    S_nominal = [x, y, z]           (world frame, meters)
Velocity:    V_nominal = [vx, vy, vz]        (body frame, m/s)
Quaternion:  q_nominal = [w, x, y, z]        (unit norm, world→body rotation)
```

**Error State** (stored in `S[]`, has covariance `P`):
```
δS = [δx, δy, δz,           // position error (world frame, meters)
      δvx, δvy, δvz,        // velocity error (body frame, m/s)
      δθx, δθy, δθz]        // attitude error (Rodrigues parameters, radians)
```

**Why Error-State?**
1. **Small errors**: Linearization is more accurate when errors remain small
2. **Quaternion handling**: Attitude error is 3D (not 4D), avoiding quaternion constraints in covariance
3. **Numerical stability**: Covariance remains well-conditioned (9×9 instead of 10×10+)
4. **Easy normalization**: Quaternion normalization doesn't affect covariance

**Error Injection** (during `kalmanCoreFinalize()`):
```c
// After measurement updates accumulate attitude error:
q_new = δq(δθ) ⊗ q_old      // Apply attitude error to quaternion
q_new = q_new / ||q_new||   // Renormalize
δθ = [0, 0, 0]              // Reset error to zero
```

---

## Coordinate Frames

The Crazyflie firmware uses multiple coordinate frames for different purposes. Understanding frame conventions is critical for correct measurement model implementation and debugging.

### 1. World Frame (Global/Inertial Frame)

**Convention**: Right-handed East-North-Up (ENU) or Forward-Left-Up (FLU) depending on initialization

**Axes**:
- **+X**: East (or Forward at initialization)
- **+Y**: North (or Left at initialization)
- **+Z**: Up (vertical, opposite gravity direction)

**Properties**:
- **Inertial**: Fixed in space, does not rotate with the drone
- **Origin**: Set at initialization (typically where the drone powers on)
- **Gravity**: Points in **-Z** direction with magnitude `g = 9.81 m/s²`

**Used For**:
- Position estimates: `S[KC_STATE_X/Y/Z]`
- Setpoints from commander/planner
- Logging and visualization
- External localization systems (MoCap, LPS anchors, Lighthouse base stations)

**Initialization**:
```c
// At startup, initial yaw sets world frame orientation:
initialYaw = 0.0       → +X faces drone's forward direction
initialYaw = π/2       → +X points left of drone
initialYaw = π         → +X points behind drone
```

### 2. Body Frame

**Convention**: Front-Left-Up (FLU) - same handedness as world frame

**Axes**:
- **+X**: Forward (nose direction)
- **+Y**: Left (left wing/motor direction)
- **+Z**: Up (opposite thrust direction)

**Properties**:
- **Body-fixed**: Rotates with the drone
- **Origin**: Center of mass (approximately geometric center)
- **Moves**: Translates and rotates with the vehicle

**Used For**:
- Velocity estimates: `S[KC_STATE_PX/PY/PZ]`
- IMU measurements (accel, gyro)
- Attitude errors: `S[KC_STATE_D0/D1/D2]`
- Aerodynamic forces and moments

**Relationship to World Frame**:
```
Quaternion q:    world → body rotation
Rotation matrix R = quat_to_matrix(q):
  v_body = R * v_world
  v_world = R^T * v_body    (R^T is the transpose/inverse)
```

### 3. Sensor Frames

Each sensor may have its own local frame relative to the body frame.

#### IMU Frame
**Typical mounting** (Crazyflie 2.x):
- **Aligned with body frame** (no rotation needed)
- **Translation**: Small offset from CoM (typically negligible)

**Measurements**:
- Accelerometer: Measures specific force in body frame [g]
- Gyroscope: Measures angular velocity in body frame [rad/s]

#### Flow Deck Frame
**Mounting**:
- **Bottom-facing** (downward looking)
- **Possible rotation**: 0° or 45° depending on deck version
- **Translation**: Below CoM by deck height (~2-3cm)

**Measurements**:
- Optical flow: Pixel velocity in sensor X/Y plane
- Conversion to body frame velocity requires:
  - Height estimate (range from ground)
  - Rotation compensation (if sensor rotated 45°)
  - Gyro de-rotation (removes rotation-induced flow)

#### Lighthouse Sensor Frame
**Mounting**:
- **4 photodiodes** on deck, each with 3D position in body frame
- **Normals**: Point in different directions for coverage
- **Coordinates**: Stored in `lighthouseCalibration.sensorPositionsAndNormals`

**Measurements**:
- Sweep angles from each base station
- Measurement model projects 3D sensor position to expected angles
- Jacobian accounts for sensor offset from body origin

#### UWB Antenna Frame
**Mounting**:
- **Single antenna** (DWM1000 chip on Loco deck)
- **Position**: Approximately at deck center, above CoM
- **Translation**: Negligible for position estimation

**Measurements**:
- TDoA: Range difference (independent of antenna position to first order)
- TWR: Absolute range (antenna position offset negligible vs. anchor distances)

### 4. Deck/Sensor Coordinate Conventions Summary

| Sensor | Frame Alignment | Translation | Rotation Handled |
|--------|-----------------|-------------|------------------|
| BMI088/MPU9250 (IMU) | Body frame | ~0 | No rotation |
| PMW3901 (Flow) | Bottom-facing | -Z offset | 0° or 45° rotation |
| VL53L0X/VL53L1X (ToF) | Downward | -Z offset | Range along -Z body axis |
| Lighthouse sensors | Deck-mounted | X/Y/Z offsets stored | Normals in body frame |
| DWM1000 (UWB) | Deck-mounted | Small offset | Negligible for ranges >1m |

---

## Covariance Matrix

The covariance matrix `P` is a **9×9 symmetric positive semi-definite matrix** representing the uncertainty of the error state.

### Structure and Interpretation

```
P = [ P_xx  P_xy  P_xz  P_xvx P_xvy P_xvz P_xθx P_xθy P_xθz ]
    [ P_yx  P_yy  P_yz  P_yvx P_yvy P_yvz P_yθx P_yθy P_yθz ]
    [   .     .     .     .     .     .     .     .     .   ]
    [   .     .     .     .     .     .     .     .     .   ]
    [ P_θzx P_θzy P_θzz P_θzvx P_θzvy P_θzvz P_θzθx P_θzθy P_θzθz ]
```

**Diagonal Elements** (variances):
```
P[0][0] = σ²_x      // Position X variance [m²]
P[1][1] = σ²_y      // Position Y variance [m²]
P[2][2] = σ²_z      // Position Z variance [m²]
P[3][3] = σ²_vx     // Velocity X variance [(m/s)²]
P[4][4] = σ²_vy     // Velocity Y variance [(m/s)²]
P[5][5] = σ²_vz     // Velocity Z variance [(m/s)²]
P[6][6] = σ²_θx     // Attitude error X variance [rad²]
P[7][7] = σ²_θy     // Attitude error Y variance [rad²]
P[8][8] = σ²_θz     // Attitude error Z variance [rad²]
```

**Off-Diagonal Elements** (covariances):
- `P[i][j]` where `i ≠ j`: Correlation between states `i` and `j`
- Example: `P[0][3]` = covariance between position-X and velocity-X
- High correlation means errors in both states tend to occur together

**Physical Meaning**:
- **Position uncertainty**: `√P[i][i]` for i=0,1,2 gives 1-sigma position error in meters
- **Velocity uncertainty**: `√P[i][i]` for i=3,4,5 gives 1-sigma velocity error in m/s
- **Attitude uncertainty**: `√P[i][i]` for i=6,7,8 gives 1-sigma attitude error in radians

### Covariance Initialization

At startup (`kalmanCoreInit`), `P` is initialized as a diagonal matrix:

```c
P[KC_STATE_X][KC_STATE_X] = stdDevInitialPosition_xy²    // ~1-100 m²
P[KC_STATE_Y][KC_STATE_Y] = stdDevInitialPosition_xy²
P[KC_STATE_Z][KC_STATE_Z] = stdDevInitialPosition_z²     // ~0.01-1 m²
P[KC_STATE_PX][KC_STATE_PX] = stdDevInitialVelocity²     // ~0.0001 (m/s)²
P[KC_STATE_PY][KC_STATE_PY] = stdDevInitialVelocity²
P[KC_STATE_PZ][KC_STATE_PZ] = stdDevInitialVelocity²
P[KC_STATE_D0][KC_STATE_D0] = stdDevInitialAttitude_rollpitch²  // ~0.0001 rad²
P[KC_STATE_D1][KC_STATE_D1] = stdDevInitialAttitude_rollpitch²
P[KC_STATE_D2][KC_STATE_D2] = stdDevInitialAttitude_yaw²        // ~0.01 rad²
```

**Rationale**:
- **Position**: High uncertainty (unknown start position)
- **Velocity**: Low uncertainty (assume stationary start)
- **Attitude (roll/pitch)**: Low uncertainty (gravity vector from accel gives initial attitude)
- **Attitude (yaw)**: Higher uncertainty (no magnetometer → yaw unobservable from accel alone)

### Covariance Propagation

**During Prediction** (`kalmanCorePredict`):
```
P = F * P * F^T + G * Q * G^T
```
where:
- `F`: State transition matrix (linearized dynamics)
- `Q`: Process noise covariance
- `G`: Process noise input matrix

**During Measurement Update** (`kalmanCoreScalarUpdate`):
```
S = H * P * H^T + R          // Innovation covariance
K = P * H^T * S^-1           // Kalman gain
P = (I - K*H) * P * (I - K*H)^T + K * R * K^T    // Joseph form
```
where:
- `H`: Measurement Jacobian
- `R`: Measurement noise variance
- Joseph form ensures `P` remains symmetric and positive semi-definite

**During Finalization** (`kalmanCoreFinalize`):
```
P = A * P * A^T    // Rotate covariance when injecting attitude error
```
where `A` is the second-order covariance rotation matrix (Mueller et al. 2016)

### Covariance Bounds and Health

**Bounds Enforcement**:
```c
MIN_COVARIANCE = 1e-6    // Prevent overconfidence
MAX_COVARIANCE = 100     // Prevent divergence

for (i = 0; i < KC_STATE_DIM; i++) {
  P[i][i] = constrain(P[i][i], MIN_COVARIANCE, MAX_COVARIANCE);
}
```

**Symmetry Enforcement**:
```c
// After every update, ensure P = (P + P^T) / 2
for (i = 0; i < KC_STATE_DIM; i++) {
  for (j = i+1; j < KC_STATE_DIM; j++) {
    float avg = (P[i][j] + P[j][i]) / 2.0f;
    P[i][j] = avg;
    P[j][i] = avg;
  }
}
```

**Health Monitoring**:
- **Divergence detection**: If `max(diag(P)) > MAX_COVARIANCE`, filter may be diverging
- **Supervisor checks**: `kalmanSupervisorIsStateWithinBounds` monitors covariance
- **Logging**: `kalman.statePX/PY/PZ` logs diagonal variances for debugging

---

## Frames and conventions
- **Global frame**: static reference frame used by controllers and setpoints. +X is forward, +Y is left, +Z is up. Gravity is aligned with negative Z (`GRAVITY_MAGNITUDE` is subtracted during prediction), meaning height increases with altitude.
- **Body frame**: fixed to the Crazyflie, +X forward, +Y left, +Z up. Velocities and accelerations coming from the IMU are expressed in this frame. `kalmanCoreExternalizeState()` rotates them into the global frame before sharing with the stabilizer.
- **Attitude errors (`D0..D2`)**: Rodrigues-parameter representation of the small difference between the current quaternion and the EKF attitude. When the error grows, `kalmanCoreFinalize()` rotates `q` by the error quaternion and resets `D*` to zero, keeping the covariance aligned with the reset attitude.
- **Yaw definition**: `initialYaw` (radians) defines the world-frame heading at boot. Yaw zero means facing +X, `π/2` facing +Y. External yaw references (Lighthouse, Mocap) provide `yawErrorMeasurement_t` that updates `D2` directly.

## Covariance interpretation
- `P` is kept symmetric and positive semi-definite by Joseph-form updates (`kalmanCoreScalarUpdate`) and by explicit symmetrization/clamping after process-noise addition and finalization.
- Diagonal entries correspond to variance of the associated state entry. The log group `kalman.var*` exposes them so you can monitor estimator certainty.
- Process noise is injected through `kalmanCoreAddProcessNoise()` and controlled by `kalmanCoreParams_t` fields (`procNoiseAcc_xy`, `procNoiseVel`, `procNoiseAtt`, etc.). These parameters are exposed as persistent `kalman.*` parameters.

## Externalized state (`state_t`)
- **Position**: `state.position.{x,y,z}` – direct copy of `S[KC_STATE_X/Y/Z]`.
- **Velocity**: rotated body-frame velocities, i.e. `R * [PX PY PZ]^T` in m/s.
- **Acceleration**: rotated accelerometer reading (in g), gravity removed from Z to match legacy expectations (`state.acc.z = world_z - 1 g`).
- **Attitude**: Euler angles in degrees (`roll` follows body X, `pitch` negative body Y to match legacy coordinate system, `yaw` positive around Z). The quaternion is exported unmodified for applications that require it.

## Units summary
- Positions, distances, heights: meters.
- Velocities: meters/second.
- Accelerations (IMU input): g (converted internally when necessary).
- Angular rates: rad/s inside the Kalman core; deck drivers convert deg/s to rad/s before calling `kalmanCorePredict()` or measurement models.
- Angles (yaw/pitch/roll, sweep errors): radians internally, degrees when published to logs.
