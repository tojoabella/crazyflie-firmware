# Lighthouse Positioning System Overview

This document provides a comprehensive overview of the Lighthouse positioning system in the Crazyflie firmware. It serves as a navigable map for developers who need to understand, modify, or debug the Lighthouse positioning pipeline.

## Quick Navigation

- **[PIPELINE.md](PIPELINE.md)**: Step-by-step flow from FPGA pulses → pulse processor → position estimator → Kalman filter
- **[CALIBRATION.md](CALIBRATION.md)**: OOTX data decoding and distortion correction models
- **[V1_VS_V2.md](V1_VS_V2.md)**: Protocol differences between Lighthouse V1 (SteamVR 1.0) and V2 (SteamVR 2.0)

## Architecture Summary

The Lighthouse positioning system uses infrared laser sweeps from SteamVR base stations to determine the Crazyflie's position in 3D space. The architecture consists of:

1. **Hardware Layer**: Lighthouse deck with 4 TS4231 photodiode sensors + FPGA for pulse timing
2. **FPGA Decoder**: Measures pulse timestamps and widths, outputs frames via UART
3. **Pulse Processor**: Decodes frames into sweep angles (V1 or V2 protocol)
4. **OOTX Decoder**: Extracts calibration data embedded in laser sweeps
5. **Position Estimator**: Converts angles to position estimates or sweep angle measurements
6. **Kalman Integration**: Feeds measurements into the Extended Kalman Filter

### System Type Selection

The firmware auto-detects Lighthouse protocol based on received pulses:
- **V1 (SteamVR 1.0)**: Sync pulses + sequential sweeps, up to 2 base stations
- **V2 (SteamVR 2.0)**: No sync pulses, channel-based identification, up to 16 base stations

See [V1_VS_V2.md](V1_VS_V2.md) for detailed protocol comparison.

## Directory Guide

- `src/deck/drivers/src/lighthouse.c`: Deck driver initialization and UART ISR
- `src/modules/src/lighthouse/`: Core processing (lighthouse_core.c, position estimation, storage)
- `src/modules/interface/lighthouse/`: Public headers for lighthouse subsystem
- `src/utils/src/lighthouse/`: Protocol-agnostic utilities (pulse processors, calibration, geometry, OOTX)
- `src/utils/interface/lighthouse/`: Headers for utility functions
- `docs/lighthouse/`: High-level documentation (this file, pipeline, calibration, protocol comparison)

## File Inventory

The table below maps every source file in the Lighthouse positioning pipeline.

| Path | Category | Key Functions / Structs | Responsibility |
|------|----------|-------------------------|----------------|
| `src/deck/drivers/src/lighthouse.c` | Deck driver | `lighthouseDeckInit`, `lighthouseTask`, UART ISR | Deck initialization, receives UART frames from FPGA, dispatches to lighthouse_core |
| `src/modules/src/lighthouse/lighthouse_core.c` | Core orchestration | `lighthouseCoreTask`, `processFrame`, `usePulseResult` | Main FreeRTOS task, protocol detection, dispatches to V1/V2 pulse processors, handles calibration and angle output |
| `src/modules/interface/lighthouse/lighthouse_core.h` | Core interface | `lighthouseCoreState`, `lighthouseCoreInit` | Public API for lighthouse core, shared state structure |
| `src/modules/src/lighthouse/lighthouse_position_est.c` | Position estimation | `lighthousePositionEstimatePoseCrossingBeams`, `lighthousePositionEstimatePoseSweeps` | Converts angles to position (crossing beams method) or enqueues sweep angles to Kalman filter |
| `src/modules/interface/lighthouse/lighthouse_position_est.h` | Position interface | `lighthousePositionEstInit`, geometry setters | Public API for position estimation |
| `src/modules/src/lighthouse/lighthouse_storage.c` | Persistent storage | `lighthouseStoragePersistData`, `lighthouseStorageGetData` | Stores/retrieves geometry and calibration from flash |
| `src/modules/interface/lighthouse/lighthouse_storage.h` | Storage interface | Storage API declarations | Public API for lighthouse data persistence |
| `src/modules/src/lighthouse/lighthouse_throttle.c` | Rate limiting | `lighthouseThrottleMeasurement` | Prevents estimator queue overflow by limiting measurement rate |
| `src/utils/src/lighthouse/pulse_processor.c` | Common utilities | `pulseProcessorClear`, `pulseProcessorApplyCalibration` | Shared pulse processor functions for V1 and V2 |
| `src/utils/interface/lighthouse/pulse_processor.h` | Pulse processor types | `pulseProcessor_t`, `pulseProcessorResult_t`, `pulseProcessorFrame_t` | Data structures for pulse processing |
| `src/utils/src/lighthouse/pulse_processor_v1.c` | V1 protocol | `pulseProcessorV1ProcessPulse`, sync detection, angle calculation | Decodes V1 sync pulses and sweeps, extracts OOTX bits |
| `src/utils/interface/lighthouse/pulse_processor_v1.h` | V1 interface | V1-specific API | V1 pulse processor declarations |
| `src/utils/src/lighthouse/pulse_processor_v2.c` | V2 protocol | `pulseProcessorV2ProcessPulse`, block processing, angle calculation | Decodes V2 channel-based frames, handles block pairing |
| `src/utils/interface/lighthouse/pulse_processor_v2.h` | V2 interface | V2-specific API | V2 pulse processor declarations |
| `src/utils/src/lighthouse/lighthouse_calibration.c` | Calibration application | `lighthouseCalibrationApplyV1`, `lighthouseCalibrationApplyV2`, `lighthouseCalibrationMeasurementModelLh1/2` | Newton-Raphson distortion correction, forward measurement models |
| `src/utils/interface/lighthouse/lighthouse_calibration.h` | Calibration types | `lighthouseCalibration_t`, `lighthouseCalibrationSweep_t` | Calibration data structures |
| `src/utils/src/lighthouse/lighthouse_geometry.c` | Geometry math | `lighthouseGeometryGetPositionFromRayIntersection`, `lighthouseGeometryGetRay` | Ray construction, line intersection, yaw calculation |
| `src/utils/interface/lighthouse/lighthouse_geometry.h` | Geometry types | `baseStationGeometry_t`, `vec3d` | Geometry data structures |
| `src/utils/src/lighthouse/ootx_decoder.c` | OOTX decoder | `ootxDecoderProcessBit`, state machine | Decodes OOTX bitstream (preamble, payload, CRC) |
| `src/utils/interface/lighthouse/ootx_decoder.h` | OOTX types | `ootxDataFrame_s`, `OotxDecoderState_t` | OOTX frame structure and decoder state |
| `src/modules/src/kalman_core/mm_sweep_angles.c` | Kalman integration | `kalmanCoreUpdateWithSweepAngles` | EKF measurement model for Lighthouse sweep angles |

## Key Concepts

### Sweep Angle Measurement

Each Lighthouse base station has rotating laser planes that sweep across the room. When a sweep hits a sensor on the Crazyflie:

1. **V1**: Time since last sync pulse → sweep angle
2. **V2**: Offset within channel period → sweep angle

The angle represents the direction from the base station to the sensor. With calibration applied, these angles become precise geometric measurements.

### Two Position Estimation Methods

**1. Crossing Beams** (`estimatePositionCrossingBeams`):
- Requires 2 base stations with valid geometry
- Computes ray intersection for each sensor
- Produces direct XYZ position measurement
- Accuracy: ~1-2 cm
- Use case: When geometry is calibrated and both base stations are visible

**2. Sweep Angles** (`estimatePositionSweeps`):
- Works with single base station
- Feeds individual angle measurements to Kalman filter
- EKF fuses with IMU for position estimation
- More robust to partial visibility
- Use case: When only one base station is visible or for smoother EKF fusion

### Calibration System

Base stations broadcast calibration parameters via OOTX (embedded in laser sweeps):
- **Phase**: Angular offset
- **Tilt**: Sweep plane tilt
- **Curve**: Lens distortion
- **Gibbs/Ogee**: Harmonic corrections (mechanical vibrations)

The calibration is applied via iterative Newton-Raphson to invert the non-linear distortion model.

See [CALIBRATION.md](CALIBRATION.md) for detailed calibration documentation.

### V1 vs V2 Protocol

| Aspect | V1 (SteamVR 1.0) | V2 (SteamVR 2.0) |
|--------|------------------|------------------|
| Sync | Sync pulse at start of each frame | No sync pulses |
| Identification | Sync pulse skip pattern | Channel (unique period) |
| Rotors | Vertical (0°), Horizontal (90°) | Tilted ±30° |
| Max Base Stations | 2 | 16 |
| Angle Calculation | Single sweep timing | Block pairing required |

See [V1_VS_V2.md](V1_VS_V2.md) for complete protocol comparison.

## Data Flow Summary

```
┌─────────────────┐
│ Base Stations   │  IR laser sweeps
└────────┬────────┘
         ▼
┌─────────────────┐
│ TS4231 Sensors  │  4 photodiodes on deck
└────────┬────────┘
         ▼
┌─────────────────┐
│ FPGA Decoder    │  Pulse timestamps & widths
└────────┬────────┘
         │ UART frames
         ▼
┌─────────────────┐
│ lighthouse_core │  Protocol detection, dispatches to V1/V2
└────────┬────────┘
         ▼
┌─────────────────┐
│ Pulse Processor │  V1 or V2 angle calculation
│ (V1 or V2)      │
└────────┬────────┘
         │ Raw angles + OOTX bits
         ▼
┌─────────────────┐
│ OOTX Decoder    │  Calibration extraction
└────────┬────────┘
         ▼
┌─────────────────┐
│ Calibration     │  Distortion correction
└────────┬────────┘
         │ Corrected angles
         ▼
┌─────────────────────────────────────────┐
│ Position Estimation                      │
├─────────────────┬───────────────────────┤
│ Crossing Beams  │    Sweep Angles       │
│ (2 BS → XYZ)    │    (1 BS → angles)    │
└────────┬────────┴───────────┬───────────┘
         │                    │
         ▼                    ▼
┌─────────────────┐   ┌─────────────────┐
│ Position Queue  │   │ Sweep Angle     │
│                 │   │ Queue           │
└────────┬────────┘   └────────┬────────┘
         │                     │
         └──────────┬──────────┘
                    ▼
┌─────────────────────────────────────────┐
│ Kalman Filter (mm_sweep_angles.c)       │
│ EKF position/orientation update         │
└─────────────────────────────────────────┘
```

See [PIPELINE.md](PIPELINE.md) for detailed data flow documentation.

## Common Debugging Questions

| Question | Answer |
|----------|--------|
| **How do I check if base stations are detected?** | Log `lighthouse.bsActive` - bitmask of active base stations |
| **How do I check calibration status?** | Log `lighthouse.bsCalVal` (calibration valid bitmap) and `lighthouse.calibConfirmed` |
| **How do I check geometry status?** | Log `lighthouse.bsGeoVal` (geometry valid bitmap) |
| **Why is position not updating?** | Check: (1) base stations detected, (2) calibration received, (3) geometry configured, (4) estimator is Kalman |
| **How do I set base station geometry?** | Via cfclient or memory module - write to lighthouse memory region |
| **What's the measurement rate?** | V1: ~60 Hz per sensor per base station, V2: ~50 Hz |
| **How do I switch estimation method?** | Parameter `lighthouse.method` (0=crossing beams, 1=sweep angles) |
| **Where are raw angles logged?** | `lighthouse.rawAngle*` log variables |
| **Why are angles drifting?** | Check calibration - may need to re-receive OOTX or reload from flash |

## Runtime Parameters

**Core Control**:
- `lighthouse.method` (uint8): Position estimation method (0=crossing beams, 1=sweep angles)
- `lighthouse.systemType` (uint8): Force system type (0=auto, 1=V1, 2=V2)

**Measurement Noise**:
- `lighthouse.sweepStd` (float): Standard deviation for V1 sweep angles (default: 0.0004 rad)
- `lighthouse.sweepStd2` (float): Standard deviation for V2 sweep angles (default: 0.001 rad)

## Log Variables

**Status**:
- `lighthouse.bsActive` (uint16): Bitmask of active base stations
- `lighthouse.bsGeoVal` (uint16): Bitmask of base stations with valid geometry
- `lighthouse.bsCalVal` (uint16): Bitmask of base stations with valid calibration

**Position (Crossing Beams)**:
- `lighthouse.x/y/z` (float): Position estimate from crossing beams method

**Rates**:
- `lighthouse.posRt` (float): Position measurement rate
- `lighthouse.estBs0Rt/estBs1Rt` (float): Per-base-station estimation rate

## Testing

**Hardware Setup**:
1. Install Lighthouse deck on Crazyflie
2. Place 1-2 SteamVR base stations with clear line of sight
3. Configure base station geometry via cfclient

**Verification Steps**:
1. Check `lighthouse.bsActive` shows detected base stations
2. Check `lighthouse.bsCalVal` shows calibration received
3. Check `lighthouse.bsGeoVal` shows geometry configured
4. Check `kalman.stateX/Y/Z` shows reasonable position

**Common Issues**:
- **No base stations detected**: Check IR line of sight, deck connection
- **Position drift**: Re-calibrate geometry, check for reflections
- **Jumpy position**: Reduce measurement noise parameter, check for multipath

---

## Next Steps

- **Understand the pipeline**: Read [PIPELINE.md](PIPELINE.md) for detailed data flow
- **Learn calibration**: Read [CALIBRATION.md](CALIBRATION.md) for OOTX and distortion correction
- **Compare protocols**: Read [V1_VS_V2.md](V1_VS_V2.md) for V1 vs V2 differences
- **Explore source code**: Use the file inventory table above to navigate
- **Tune parameters**: Experiment with `lighthouse.sweepStd` via cfclient

---

**Document Version**: 1.0
**Last Updated**: 2026-02-12
**Maintainers**: Crazyflie firmware contributors
