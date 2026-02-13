# Lighthouse Calibration System

This document describes the calibration system for the Lighthouse positioning system, including OOTX data decoding and the distortion correction models.

## Overview

Lighthouse base stations have manufacturing imperfections in their optical systems that introduce systematic errors in angle measurements. Each base station broadcasts its calibration parameters via an embedded data channel called **OOTX** (On-the-air Over-The-eXtended-protocol). The Crazyflie receives these parameters and applies a distortion correction model to convert raw measured angles to accurate geometric angles.

---

## OOTX Data Channel

### What is OOTX?

OOTX is a slow data channel embedded in the laser sweeps. Each sweep carries one bit of data, allowing the base station to transmit calibration and identification information over time.

**Bit Rate**:
- V1: ~120 bits/second (one bit per 8.33ms frame)
- V2: ~1.4 bits/second per channel (sparse transmission)

**Full Frame Time**:
- V1: ~3-4 seconds to receive complete calibration
- V2: Similar, but varies by channel utilization

### Bit Extraction

**V1 Protocol** ([pulse_processor_v1.c](../../src/utils/src/lighthouse/pulse_processor_v1.c)):
```c
// OOTX bit encoded in sync pulse width
// Bit 1 of the width offset indicates OOTX data
uint8_t ootxBit = ((syncPulseWidth - SYNC_BASE_WIDTH) >> 1) & 0x01;
ootxDecoderProcessBit(&ootxDecoder[baseStation], ootxBit);
```

**V2 Protocol** ([pulse_processor_v2.c](../../src/utils/src/lighthouse/pulse_processor_v2.c)):
```c
// OOTX bit directly available in frame
uint8_t ootxBit = frame.slowBit;
ootxDecoderProcessBit(&ootxDecoder[channel], ootxBit);
```

---

## OOTX Frame Format

### Frame Structure

```
┌─────────────┬────────────────────┬─────────────┐
│ Preamble    │ Payload            │ CRC         │
│ 17 zeros +1 │ Length + Data      │ 32-bit      │
└─────────────┴────────────────────┴─────────────┘
```

**Preamble**: 17 or more consecutive zeros followed by a single '1' bit
**Payload**: 16-bit length field followed by data bytes
**CRC**: CRC32 checksum (polynomial 0x04C11DB7)

### Bit Stuffing

After every 16 payload bits, a stuffing '1' bit is inserted:
```
[16 data bits] [1 stuffing] [16 data bits] [1 stuffing] ...
```

If a stuffing bit is '0', it indicates a framing error.

### Decoder State Machine ([ootx_decoder.c](../../src/utils/src/lighthouse/ootx_decoder.c))

```c
typedef enum {
    rxWaitForSync,   // Waiting for preamble (17+ zeros then 1)
    rxLength,        // Receiving frame length
    rxData,          // Receiving payload data
    rxCrc0,          // Receiving first CRC word
    rxCrc1,          // Receiving second CRC word
    rxDone           // Frame complete, validate CRC
} OotxDecoderState_t;
```

**State Transitions**:
```
         ┌────────────────────────────────────────┐
         │                                        │
         v                                        │
    rxWaitForSync ──17+ zeros, then 1──> rxLength │
         │                                        │
         │ framing error                          │
         └────────────────────────────────────────┘

    rxLength ──16 bits + stuffing──> rxData
                                         │
    rxData ──payload complete──> rxCrc0 ──> rxCrc1 ──> rxDone
                                                          │
                                                          v
                                               CRC OK? Return frame
                                               CRC fail? → rxWaitForSync
```

---

## OOTX Payload Fields

### Frame Data Structure ([ootx_decoder.h](../../src/utils/interface/lighthouse/ootx_decoder.h))

```c
struct ootxDataFrame_s {
    // Identification
    uint8_t protocolVersion : 6;
    uint16_t firmwareVersion : 10;
    uint32_t id;              // Unique base station ID

    // Calibration - Sweep 0 (X rotor)
    __fp16 phase0;            // Phase offset
    __fp16 tilt0;             // Tilt angle
    __fp16 curve0;            // Curvature coefficient
    __fp16 gibmag0;           // Gibbs magnitude (2nd harmonic)
    __fp16 gibphase0;         // Gibbs phase
    __fp16 ogeemag0;          // Ogee magnitude (3rd harmonic, V2 only)
    __fp16 ogeephase0;        // Ogee phase (V2 only)

    // Calibration - Sweep 1 (Y rotor)
    __fp16 phase1;
    __fp16 tilt1;
    __fp16 curve1;
    __fp16 gibmag1;
    __fp16 gibphase1;
    __fp16 ogeemag1;          // V2 only
    __fp16 ogeephase1;        // V2 only

    // Diagnostics
    uint8_t unlockCount;
    uint8_t hwVersion;
    int8_t accelX, accelY, accelZ;  // Accelerometer (orientation check)
    uint8_t mode;
    uint8_t faults;
};
```

### Field Meanings

| Field | Units | Description |
|-------|-------|-------------|
| `phase` | radians | Angular offset of sweep plane |
| `tilt` | radians | Tilt of the sweep plane |
| `curve` | radians/radian² | Quadratic curvature correction |
| `gibmag` | radians | Magnitude of 2nd harmonic error |
| `gibphase` | radians | Phase of 2nd harmonic error |
| `ogeemag` | radians | Magnitude of 3rd harmonic error (V2) |
| `ogeephase` | radians | Phase of 3rd harmonic error (V2) |

---

## Distortion Correction Model

### Purpose

The raw sweep angles measured by the sensors are distorted by:
1. **Phase offset**: Fixed angular bias
2. **Tilt**: Sweep plane not perpendicular to rotation axis
3. **Curvature**: Lens-induced polynomial distortion
4. **Gibbs effect**: 2nd harmonic oscillation (mechanical vibration)
5. **Ogee effect**: 3rd harmonic oscillation (V2 only)

### Forward Model

The forward model predicts the distorted angle that would be measured given an ideal angle:

**V1 Model** ([lighthouse_calibration.c](../../src/utils/src/lighthouse/lighthouse_calibration.c)):
```c
float lighthouseCalibrationMeasurementModelLh1(
    float x, float y, float z, float t,
    const lighthouseCalibrationSweep_t* calib)
{
    // Base angle from 3D point
    float angle = atan2f(y, x);

    // Apply distortion
    angle += calib->phase;
    angle += calib->tilt * z / hypotf(x, y);
    angle += calib->curve * (z * z) / (x * x);
    angle += calib->gibmag * sinf(angle + calib->gibphase);

    return angle;
}
```

**V2 Model**:
```c
float lighthouseCalibrationMeasurementModelLh2(
    float x, float y, float z, float t,
    const lighthouseCalibrationSweep_t* calib)
{
    // Account for tilted rotor (t = ±30°)
    float ct = cosf(t);
    float st = sinf(t);

    // Rotate point to rotor frame
    float y_rot = ct * y - st * z;
    float z_rot = st * y + ct * z;

    // Base angle
    float angle = atan2f(y_rot, x);

    // Apply distortion (same as V1 plus ogee)
    angle += calib->phase;
    angle += calib->tilt * z_rot / hypotf(x, y_rot);
    angle += calib->curve * (z_rot * z_rot) / (x * x);
    angle += calib->gibmag * sinf(angle + calib->gibphase);
    angle += calib->ogeemag * sinf(2 * angle + calib->ogeephase);  // 3rd harmonic

    return angle;
}
```

### Inverse Model (Calibration Application)

To correct a measured angle, we need the inverse of the distortion model. Since the model is non-linear, we use iterative Newton-Raphson:

```c
void lighthouseCalibrationApply(
    const lighthouseCalibration_t* calib,
    const float* rawAngles,
    float* correctedAngles)
{
    const float maxError = 0.0005;  // radians (~0.03°)

    // Initialize with raw (distorted) angles
    float estimated[2] = {rawAngles[0], rawAngles[1]};

    for (int iter = 0; iter < 5; iter++) {
        // Compute what distorted angles we'd get from current estimate
        float predicted[2];
        idealToDistorted(calib, estimated, predicted);

        // Update estimate based on error
        float delta0 = rawAngles[0] - predicted[0];
        float delta1 = rawAngles[1] - predicted[1];

        estimated[0] += delta0;
        estimated[1] += delta1;

        // Check convergence
        if (fabsf(delta0) < maxError && fabsf(delta1) < maxError) {
            break;
        }
    }

    correctedAngles[0] = estimated[0];
    correctedAngles[1] = estimated[1];
}
```

**Convergence**: Typically converges in 2-3 iterations for typical calibration values.

---

## Calibration Data Flow

### Receiving Calibration

```
OOTX bit received
  │
  v
ootxDecoderProcessBit()
  │
  ├─ Accumulate bits
  │
  └─ When frame complete:
       │
       v
     lighthouseCalibrationInitFromFrame()
       │
       ├─ Extract sweep[0] calibration (X rotor)
       ├─ Extract sweep[1] calibration (Y rotor)
       ├─ Store base station UID
       └─ Set calib.valid = true
```

### Storage and Persistence

Calibration data can be:
1. **Received over the air** via OOTX (automatic)
2. **Loaded from flash** on startup ([lighthouse_storage.c](../../src/modules/src/lighthouse/lighthouse_storage.c))
3. **Written via memory module** from PC client

```c
// In lighthouse_core.c
if (calibDataIsDecoded) {
    if (calibrationDiffersFromStored) {
        lighthouseStoragePersistCalibDataBackground(baseStation, &newCalib);
    }
    modifyBit(&baseStationCalibConfirmedMap, baseStation, true);
}
```

---

## Calibration in the Kalman Filter

When sweep angle measurements are sent to the Kalman filter, the calibration model is included as a function pointer:

```c
sweepAngleMeasurement_t measurement = {
    .measuredSweepAngle = correctedAngle,
    // ...
    .calib = &bsCalibration[bs].sweep[axis],
    .calibrationMeasurementModel = lighthouseCalibrationMeasurementModelLh2,
    .t = (axis == 0) ? -M_PI/6 : M_PI/6
};

estimatorEnqueueSweepAngles(&measurement);
```

The Kalman filter uses this model in its measurement update to compute the Jacobian:

```c
// In mm_sweep_angles.c
float predictedAngle = measurement->calibrationMeasurementModel(
    x, y, z, measurement->t, measurement->calib);

// Numerical Jacobian computation
float h[3];
for (int i = 0; i < 3; i++) {
    float perturbed = measurement->calibrationMeasurementModel(
        x + dx[i], y + dy[i], z + dz[i], measurement->t, measurement->calib);
    h[i] = (perturbed - predictedAngle) / delta;
}
```

---

## Typical Calibration Values

| Parameter | Typical Range | Physical Meaning |
|-----------|---------------|------------------|
| `phase` | -0.01 to +0.01 rad | Rotational offset of laser |
| `tilt` | -0.005 to +0.005 rad | Sweep plane tilt |
| `curve` | -0.001 to +0.001 | Lens curvature |
| `gibmag` | 0 to 0.001 rad | Mechanical vibration amplitude |
| `gibphase` | 0 to 2π rad | Phase of vibration |
| `ogeemag` | 0 to 0.0005 rad | Higher-order vibration (V2) |
| `ogeephase` | 0 to 2π rad | Phase of ogee term (V2) |

---

## Debugging Calibration Issues

### Symptoms of Bad Calibration

1. **Position drift**: Slow drift when stationary
2. **Position offset**: Consistent offset from true position
3. **Axis-dependent errors**: Errors that depend on orientation
4. **Periodic oscillations**: Position oscillates with rotor frequency

### Verification Steps

1. **Check OOTX reception**:
   ```
   LOG: lighthouse.calibConfirmed = 0x03 (both base stations)
   ```

2. **Compare with stored calibration**:
   ```
   LOG: lighthouse.calibUpdated = 0x00 (no mismatch)
   ```

3. **Monitor raw vs corrected angles**:
   - Log `angles` and `correctedAngles` in pulse processor
   - Difference should be small (< 0.01 rad)

4. **Check calibration validity**:
   ```c
   if (!calib->valid) {
       DEBUG_PRINT("Warning: Using uncalibrated angles\n");
   }
   ```

---

## File Reference

| File | Purpose |
|------|---------|
| [ootx_decoder.c](../../src/utils/src/lighthouse/ootx_decoder.c) | OOTX bitstream decoder |
| [ootx_decoder.h](../../src/utils/interface/lighthouse/ootx_decoder.h) | Frame structure definitions |
| [lighthouse_calibration.c](../../src/utils/src/lighthouse/lighthouse_calibration.c) | Distortion models |
| [lighthouse_calibration.h](../../src/utils/interface/lighthouse/lighthouse_calibration.h) | Calibration types |
| [lighthouse_storage.c](../../src/modules/src/lighthouse/lighthouse_storage.c) | Flash persistence |

---

## See Also

- [PIPELINE.md](PIPELINE.md) - Complete data flow overview
- [V1_VS_V2.md](V1_VS_V2.md) - Protocol differences affecting calibration
