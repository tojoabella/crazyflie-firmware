# Sensor Ingestion

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
| Lighthouse sweep angles | `MeasurementTypeSweepAngle` | `kalmanCoreUpdateWithSweepAngles()` | Converts sweep angles from the rotor frame to world-frame constraints, gated by `outlierFilterLighthouse`. Requires geometry from the Lighthouse deck (
`rotorRot`, `sensorPos`, etc.). |
| Lighthouse yaw error | `MeasurementTypeYawError` | `kalmanCoreUpdateWithYawError()` | Injects a yaw correction derived from Lighthouse crossing-beams into the attitude error state `D2`. |
| Flow deck derived height | `MeasurementTypeFlow` (side channel) | `kalmanCoreUpdateWithFlow()` | Uses flow + range to compute velocity and implicitly constrains height through the Jacobian. |

Other helpers:
- `MeasurementTypeDistance` also has a `kalmanCoreRobustUpdateWithDistance()` path for outlier rejection.
- `MeasurementTypePose` updates both position (`stdDevPos`) and orientation (`stdDevQuat`).
- `measurement_t` definitions are in `src/modules/interface/estimator/estimator.h` and ultimately defined in `stabilizer_types.h`.
- Every measurement handler uses `kalmanCoreScalarUpdate()` internally except for the robust variants, which pre-compute their own gain/covariance and call `kalmanCoreUpdateWithPKE()`.
