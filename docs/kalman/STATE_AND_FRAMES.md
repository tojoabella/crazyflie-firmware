# State Vector, Covariance, and Frames

## State vector ordering (`kalmanCoreStateIdx_t`)
| Index | Symbol | Meaning | Units | Frame |
| --- | --- | --- | --- | --- |
| `KC_STATE_X` | x | Position X | meters | Global (Crazyflie world frame, +X forward) |
| `KC_STATE_Y` | y | Position Y | meters | Global (+Y left) |
| `KC_STATE_Z` | z | Position Z | meters | Global (+Z up) |
| `KC_STATE_PX` | v<sub>x</sub> | Body-frame velocity along +X | m/s | Body |
| `KC_STATE_PY` | v<sub>y</sub> | Body-frame velocity along +Y | m/s | Body |
| `KC_STATE_PZ` | v<sub>z</sub> | Body-frame velocity along +Z | m/s | Body |
| `KC_STATE_D0` | δ<sub>roll</sub> | Small attitude error about body X | radians | Body (Rodrigues parameters) |
| `KC_STATE_D1` | δ<sub>pitch</sub> | Small attitude error about body Y | radians | Body |
| `KC_STATE_D2` | δ<sub>yaw</sub> | Small attitude error about body Z | radians | Body |

Additional internal members in `kalmanCoreData_t`:
- `q[4]` quaternion `[w, x, y, z]` representing the full attitude.
- `R[3][3]` rotation matrix world←body derived from `q` during `kalmanCoreFinalize()`.
- `P[KC_STATE_DIM][KC_STATE_DIM]` covariance matrix in the same ordering as the state vector.
- `baroReferenceHeight`, `initialQuaternion`, time stamps and flags used by predict/finalize.

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
