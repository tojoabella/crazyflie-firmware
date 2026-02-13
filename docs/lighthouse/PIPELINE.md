# Lighthouse Positioning Pipeline: Sensors → Pulse Processing → Estimator

This document describes the complete data flow through the Crazyflie Lighthouse positioning system, from raw IR sensor pulses to position measurements consumed by the state estimator.

## Overview

The Lighthouse positioning system uses infrared laser sweeps from external base stations (SteamVR/Valve Lighthouse) to determine the Crazyflie's 3D position. The pipeline is designed as a **single-threaded, event-driven architecture**:

1. **FPGA decoder** on the Lighthouse deck detects IR pulses and sends 12-byte frames via UART
2. **Lighthouse core task** (`lighthouseCoreTask`) receives frames, decodes sync/sweep timing
3. **Pulse processor** converts raw pulse timing to sweep angles (V1 or V2 protocol)
4. **OOTX decoder** extracts calibration data embedded in the laser sweeps
5. **Calibration module** corrects angles for lens distortion
6. **Position estimator** converts corrected angles to position/yaw measurements
7. **Kalman filter** fuses measurements with IMU data for state estimation

**Key Design Principles**:
- **Protocol agnostic**: Same pipeline handles both Lighthouse V1 and V2 base stations
- **Self-calibrating**: Base station calibration data decoded from OOTX bitstream
- **Low latency**: Sweep angles sent directly to estimator without buffering
- **Robust**: Outlier filtering and health monitoring for sensor failures

---

## High-Level Call Graph

### Lighthouse Core Task (Main Processing Thread)

```
lighthouseCoreTask()  [FreeRTOS task "LH", runs continuously]
  └─ waitForUartSynchFrame()           # Synchronize to FPGA frame stream
  └─ while(1):
       ├─ getUartFrameRaw(&frame)      # Read 12-byte UART frame from FPGA
       │    └─ uart1GetDataWithTimeout()
       │
       ├─ if (frame.isSyncFrame):
       │    └─ continue                # Skip sync frames (0xFF 0xFF 0xFF...)
       │
       ├─ processFrame(&frame.data)    # Main pulse processing
       │    ├─ pulseProcessorProcessPulse()  # V1 or V2 decoder (function pointer)
       │    │    └─ pulseProcessorV1ProcessPulse() or pulseProcessorV2ProcessPulse()
       │    │
       │    ├─ if (calibDataIsDecoded):
       │    │    └─ useCalibrationData(&lighthouseCoreState)  # Store OOTX calib
       │    │
       │    ├─ if (anglesMeasured):
       │    │    ├─ pulseProcessorApplyCalibration()  # Distortion correction
       │    │    ├─ lighthouseThrottle()              # Rate limiting (V2)
       │    │    └─ usePulseResultSweeps()            # Send to estimator
       │    │         └─ lighthousePositionEstimatePoseSweep()
       │    │              ├─ estimatorEnqueueSweepAngles()  # Angle measurements
       │    │              └─ estimatorEnqueueYawError()     # Yaw correction
       │    │
       │    └─ updateSystemStatus()    # Update LED indicators
       │
       └─ lighthouseTransmitProcessFrame()  # Optional: send raw data via CRTP
```

### Pulse Processing (V1 Protocol)

```
pulseProcessorV1ProcessPulse()
  ├─ classifyPulse(&frame)             # Determine: sync0, sync1, or sweep
  │    └─ Based on pulse width thresholds
  │
  ├─ if (!synchronized):
  │    └─ synchronize()                # Build pulse history, find sync pattern
  │         ├─ storePulse(&pulseHistory)
  │         ├─ findSyncTime()          # Locate sync pulse timing
  │         └─ synchronized = true when pattern found
  │
  ├─ if (pulseClass == sync0 or sync1):
  │    ├─ extractOotxBit()             # Get OOTX data bit from pulse width
  │    │    └─ ootxDecoderProcessBit(&ootxDecoder[bs], bit)
  │    ├─ updateSyncTimestamps()       # Track frame timing
  │    └─ determineBaseStationAndAxis()
  │
  └─ if (pulseClass == sweep):
       ├─ storeSweepTimestamp()        # Save timestamp per sensor
       └─ if (allSensorsReceived):
            ├─ calculateAngles()       # Convert timing to angles
            │    angle = (sweepTs - syncTs) * 2π / frameWidth
            └─ return true, angles, baseStation, axis
```

### Pulse Processing (V2 Protocol)

```
pulseProcessorV2ProcessPulse()
  ├─ pulseProcessorV2ProcessWorkspace()
  │    ├─ if (pulse too far from previous):
  │    │    └─ processWorkspace()      # Process accumulated pulses
  │    └─ addPulseToWorkspace()        # Buffer pulse data
  │
  ├─ processWorkspace()
  │    ├─ augmentFramesInWorkspace()   # Fill missing channel info
  │    └─ processWorkspaceBlock()      # Extract sweep block
  │         ├─ extractBlockFromWorkspace()
  │         │    └─ Calculate channel, offsets, timestamp0
  │         └─ handleBlock()           # Pair blocks for angle calculation
  │              ├─ Store block in blocks[channel]
  │              └─ if (paired with previous block):
  │                   └─ calculateAnglesFromBlocks()
  │                        ├─ firstBeam = offset * 2π / period - π + π/3
  │                        ├─ secondBeam = offset * 2π / period - π - π/3
  │                        └─ return angles for all 4 sensors
  │
  └─ processOotxFromWorkspace()        # Extract OOTX bits
       └─ ootxDecoderProcessBit(&ootxDecoder[channel], slowBit)
```

---

## Detailed Pipeline Phases

### Phase 1: UART Frame Reception

**Purpose**: Receive raw pulse data from FPGA decoder on Lighthouse deck.

**Hardware Path**:
```
IR Photodiode → FPGA Pulse Detector → UART1 → STM32
```

**Frame Format** (12 bytes):
```c
typedef struct {
  uint8_t sensor;        // Sensor index (0-3)
  uint32_t timestamp;    // 24-bit, 24 MHz clock
  uint16_t width;        // V1: pulse width in ticks
  uint32_t beamData;     // V2: encoded beam info
  uint32_t offset;       // V2: rotor angle offset
  uint8_t channel;       // V2: 0-15 (base station channel)
  uint8_t slowBit;       // V2: OOTX data bit
  bool channelFound;     // V2: channel validity flag
} pulseProcessorFrame_t;
```

**Synchronization**:
- FPGA sends sync frames (0xFF 0xFF 0xFF...) periodically
- `waitForUartSynchFrame()` waits for sync pattern before processing
- Ensures byte-alignment of 12-byte frames

---

### Phase 2: Pulse Classification & Sync Detection

**Purpose**: Distinguish sync pulses from sweep pulses, establish timing reference.

**V1 Sync Detection**:
```c
// Pulse width ranges (24 MHz clock ticks)
#define SYNC_BASE_WIDTH     1250   // Nominal sync width
#define SYNC_MIN_WIDTH      1125   // Minimum valid sync
#define SYNC_MAX_WIDTH      2250   // Maximum valid sync
#define SWEEP_MAX_WIDTH      512   // Maximum sweep pulse width

// Classification logic
if (width >= SYNC_MIN_WIDTH && width <= SYNC_MAX_WIDTH) {
    // Sync pulse - extract axis and OOTX bit from width
    axis = (width - SYNC_BASE_WIDTH) & 0x04;  // Bit 2
    ootxBit = (width - SYNC_BASE_WIDTH) & 0x02;  // Bit 1
} else if (width < SWEEP_MAX_WIDTH) {
    // Sweep pulse
}
```

**V2 Channel Detection**:
- Channel encoded directly in `beamData` field
- No sync pulse needed; channel identifies base station
- `channelFound` flag indicates successful decoding

---

### Phase 3: Angle Calculation

**Purpose**: Convert pulse timing to angular measurements.

**V1 Angle Formula**:
```c
// Time from sync to sweep detection
uint32_t delta = TS_DIFF(sweepTimestamp, syncTimestamp);

// Frame width = time between two consecutive sync0 pulses
float frameWidth = currentSync0 - previousSync0;

// Angle in radians (-π/2 to +π/2 range)
float angle = (delta - frameWidth/2) * 2 * M_PI / frameWidth;
```

**V2 Angle Formula**:
```c
// Rotor period depends on channel (0-15)
static const uint32_t CYCLE_PERIODS[16] = {
    959000, 957000, ..., 887000  // Ticks per rotation
};

// Two beams per rotor rotation, tilted ±30°
float period = CYCLE_PERIODS[channel];
float firstBeam = (offset * 2 * M_PI / period) - M_PI + M_PI/3;
float secondBeam = (offset * 2 * M_PI / period) - M_PI - M_PI/3;
```

**Output Structure**:
```c
typedef struct {
  float angles[2];           // Raw X and Y angles (radians)
  float correctedAngles[2];  // After calibration
  int validCount;            // 0, 1, or 2 valid sweeps
} pulseProcessorSensorMeasurement_t;
```

---

### Phase 4: OOTX Calibration Decoding

**Purpose**: Extract base station calibration data embedded in laser sweeps.

**OOTX Bit Source**:
- V1: Encoded in sync pulse width (bit 1 of width offset)
- V2: Direct `slowBit` field in frame data

**Decoder State Machine** ([ootx_decoder.c](../../src/utils/src/lighthouse/ootx_decoder.c)):
```
States:
  1. waitForSync    - Wait for 17+ consecutive zeros then '1'
  2. rxLength       - Receive frame length (16 bits + stuffing)
  3. rxData         - Receive payload data
  4. rxCrc          - Receive CRC32 (2 × 16-bit words)
  5. rxDone         - Validate CRC and signal completion

Bit Stuffing:
  - After every 16 data bits, expect a '1' stuffing bit
  - If stuffing bit is '0' → framing error, resynchronize
```

**Decoded Calibration Data**:
```c
typedef struct {
  float phase;      // Phase offset
  float tilt;       // Plane tilt angle
  float curve;      // Polynomial curvature coefficient
  float gibmag;     // 2nd harmonic magnitude
  float gibphase;   // 2nd harmonic phase
  float ogeemag;    // 3rd harmonic magnitude (V2 only)
  float ogeephase;  // 3rd harmonic phase (V2 only)
} lighthouseCalibrationSweep_t;

typedef struct {
  lighthouseCalibrationSweep_t sweep[2];  // X and Y rotors
  uint32_t uid;                            // Base station ID
  bool valid;
} lighthouseCalibration_t;
```

---

### Phase 5: Calibration Application

**Purpose**: Correct raw angles for lens distortion and mechanical imperfections.

**Distortion Model** ([lighthouse_calibration.c](../../src/utils/src/lighthouse/lighthouse_calibration.c)):

The base station optics introduce systematic errors that must be corrected. The calibration uses an iterative Newton-Raphson approach:

```c
// Goal: Find ideal angles that, when distorted, match the measured angles
for (int i = 0; i < 5; i++) {
    float distorted[2];
    idealToDistorted(calib, estimated, distorted);

    // Newton-Raphson update
    float delta[2] = {rawAngles[0] - distorted[0],
                      rawAngles[1] - distorted[1]};
    estimated[0] += delta[0];
    estimated[1] += delta[1];

    if (fabs(delta[0]) < 0.0005 && fabs(delta[1]) < 0.0005) break;
}
```

**V1 Distortion Function**:
```c
float lighthouseCalibrationMeasurementModelLh1(x, y, z, t, calib) {
    float angle = atan2f(y, x);
    // Apply corrections: phase, tilt, curve, gibmag/gibphase
    angle += calib->phase;
    angle += calib->tilt * z / hypotf(x, y);
    angle += calib->curve * z * z / (x * x);
    angle += calib->gibmag * sinf(angle + calib->gibphase);
    return angle;
}
```

**V2 Distortion Function**:
```c
float lighthouseCalibrationMeasurementModelLh2(x, y, z, t, calib) {
    // Similar to V1 but includes:
    // - Rotor tilt angle (t = ±30°)
    // - Additional ogee (3rd harmonic) term
    angle += calib->ogeemag * sinf(2 * angle + calib->ogeephase);
}
```

---

### Phase 6: Position Estimation

**Purpose**: Convert calibrated sweep angles to position/orientation measurements.

**Two Estimation Methods** ([lighthouse_position_est.c](../../src/modules/src/lighthouse/lighthouse_position_est.c)):

**Method 0: Crossing Beams (Ray Intersection)**
```c
// Get ray from each base station through sensor
vec3d ray0 = lighthouseGeometryGetRay(&bsGeo[0], angles[0]);
vec3d ray1 = lighthouseGeometryGetRay(&bsGeo[1], angles[1]);

// Find closest point between rays
vec3d position = rayRayIntersection(bsGeo[0].origin, ray0,
                                     bsGeo[1].origin, ray1);

// Send to estimator
positionMeasurement_t measurement = {
    .x = position.x, .y = position.y, .z = position.z,
    .stdDev = 0.01,
    .source = MeasurementSourceLighthouse
};
estimatorEnqueuePosition(&measurement);
```

**Method 1: Sweep Angles (Default)**
```c
// Send raw sweep angle to Kalman filter
sweepAngleMeasurement_t sweepInfo = {
    .measuredSweepAngle = correctedAngle,
    .sensorId = sensor,
    .baseStationId = baseStation,
    .sweepId = axis,
    .sensorPos = sensorPositions[sensor],      // Deck-relative position
    .rotorPos = bsGeometry[bs].origin,          // Base station position
    .rotorRot = bsGeoCache[bs].mat,             // Base station orientation
    .calib = &bsCalibration[bs].sweep[axis],
    .t = (axis == 0) ? -M_PI/6 : M_PI/6,        // V2 rotor tilt
    .stdDev = 4e-4                               // Measurement noise
};
estimatorEnqueueSweepAngles(&sweepInfo);
```

**Yaw Error Estimation**:
```c
// Calculate yaw error from diagonal sensor pairs
float yawError = lighthouseGeometryYawDelta(&bsGeo, angles, sensorPositions);

yawErrorMeasurement_t yawMeasurement = {
    .yawError = yawError,
    .stdDev = 0.01
};
estimatorEnqueueYawError(&yawMeasurement);
```

---

## Threading Model

### Task Structure

**Single Main Task**:
```
lighthouseCoreTask  [FreeRTOS task "LH"]
  ├─ Priority: LIGHTHOUSE_TASK_PRI
  ├─ Stack: LIGHTHOUSE_TASK_STACKSIZE
  └─ Runs continuously, blocking on UART data
```

**LED Timer** (Hardware Timer):
```
ledTimerHandle  [200ms periodic]
  └─ lighthouseCoreLedTimer()
       └─ lighthouseCoreSetLeds(red, orange, green)
```

### Synchronization

- **No mutexes needed**: Single-threaded processing
- **No queues**: Pulses processed immediately as they arrive
- **Estimator queue**: Thread-safe enqueue to measurement queue

### Status Bitmaps

```c
// Updated each processing cycle
baseStationAvailabledMap      // Which base stations are configured
baseStationReceivedMap        // Received pulses this cycle
baseStationActiveMap          // Has valid geo + calib + receiving
baseStationCalibConfirmedMap  // Got OOTX calibration data
baseStationCalibUpdatedMap    // OOTX differs from stored

// LED status indication
systemStatus:
  - statusNotReceiving   // No pulses (red LED)
  - statusMissingData    // Missing geo/calib (orange LED)
  - statusToEstimator    // Working normally (green LED)
```

---

## Sensor Deck Geometry

### Physical Layout

```
Lighthouse Deck (top view):
         Front
    +-------------+
    |  [2]   [3]  |
    |             |
    |  [0]   [1]  |
    +-------------+
         Back

Sensor positions (meters, deck-relative):
  Sensor 0: (-0.015, +0.0075, 0)  // Back-left
  Sensor 1: (-0.015, -0.0075, 0)  // Back-right
  Sensor 2: (+0.015, +0.0075, 0)  // Front-left
  Sensor 3: (+0.015, -0.0075, 0)  // Front-right
```

### Coordinate Transforms

```c
// Transform sensor position from deck frame to world frame
vec3d worldPos = lighthouseGeometryGetSensorPosition(
    &state->position,           // Crazyflie world position
    &state->attitudeQuaternion, // Crazyflie orientation
    &sensorPos                  // Sensor deck-relative position
);

// Get ray direction from sweep angles
vec3d ray = lighthouseGeometryGetRay(
    &bsGeometry,    // Base station pose
    sweepAngles     // Calibrated sweep angles
);
```

---

## Timing Constants

### V1 System (24 MHz clock)

| Constant | Value | Description |
|----------|-------|-------------|
| `FRAME_LENGTH` | 200,000 ticks | 8.333 ms frame period |
| `SYNC_BASE_WIDTH` | 1,250 ticks | Nominal sync pulse width |
| `SYNC_SEPARATION` | 10,000 ticks | Time between sync0 and sync1 |
| `SWEEP_MAX_WIDTH` | 512 ticks | Maximum sweep pulse width |

### V2 System (24 MHz clock)

| Constant | Value | Description |
|----------|-------|-------------|
| `CYCLE_PERIODS[0]` | 959,000 ticks | Channel 0 rotor period |
| `CYCLE_PERIODS[15]` | 887,000 ticks | Channel 15 rotor period |
| `MAX_TICKS_SENSOR_TO_SENSOR` | 10,000 ticks | Max time between sensor hits |
| `MAX_TICKS_BETWEEN_SWEEP_STARTS` | 10 ticks | Max jitter for paired blocks |

### Timestamp Handling

```c
#define PULSE_PROCESSOR_TIMESTAMP_BITWIDTH 24
#define PULSE_PROCESSOR_TIMESTAMP_MAX      ((1 << 24) - 1)  // 16,777,215

// Wrap-safe timestamp difference
inline uint32_t TS_DIFF(uint32_t x, uint32_t y) {
    return (x - y) & PULSE_PROCESSOR_TIMESTAMP_BITMASK;
}
```

---

## Measurement Integration with Kalman Filter

### Sweep Angle Measurements

```c
// In mm_sweep_angles.c (Kalman measurement model)
void kalmanCoreUpdateWithSweepAngles(kalmanCoreData_t* this,
                                      sweepAngleMeasurement_t* angles) {
    // 1. Predict sweep angle from current state estimate
    vec3d sensorWorldPos = transformSensorToWorld(this->S, angles->sensorPos);
    float predictedAngle = calculatePredictedSweepAngle(
        sensorWorldPos, angles->rotorPos, angles->rotorRot);

    // 2. Apply calibration model to prediction
    predictedAngle = angles->calibrationMeasurementModel(
        predictedAngle, angles->calib, angles->t);

    // 3. Compute innovation
    float innovation = angles->measuredSweepAngle - predictedAngle;

    // 4. Kalman update
    kalmanCoreScalarUpdate(this, H, innovation, angles->stdDev);
}
```

### Yaw Error Measurements

```c
// Corrects yaw drift from gyro integration
void kalmanCoreUpdateWithYawError(kalmanCoreData_t* this,
                                   yawErrorMeasurement_t* yawError) {
    // Direct correction to attitude error state
    float H[KC_STATE_DIM] = {0};
    H[KC_STATE_D2] = 1.0f;  // Yaw component of attitude error

    kalmanCoreScalarUpdate(this, H, yawError->yawError, yawError->stdDev);
}
```

---

## Health Monitoring

### Sensor Health Check

```c
// On startup, verify all 4 sensors are working
#define MAX_WAIT_TIME_FOR_HEALTH_MS 4000

// Track which sensors have been seen
healthSensorBitField |= (1 << frame.sensor);

// Check timeout
if (healthSensorBitField == 0x0F) {
    healthDetermined = true;  // All 4 sensors working
} else if (elapsed > MAX_WAIT_TIME_FOR_HEALTH_MS) {
    DEBUG_PRINT("Warning: Only %d sensors detected\n",
                __builtin_popcount(healthSensorBitField));
    healthDetermined = true;  // Continue with reduced sensors
}
```

### Outlier Filtering

Outlier filtering is performed in the Kalman filter measurement models, not in the lighthouse pipeline itself. Invalid angles (NaN, out of range) are rejected before enqueueing.

---

## File Reference

| File | Purpose |
|------|---------|
| [lighthouse_core.c](../../src/modules/src/lighthouse/lighthouse_core.c) | Main task, UART parsing, frame dispatch |
| [lighthouse_position_est.c](../../src/modules/src/lighthouse/lighthouse_position_est.c) | Position/yaw estimation, geometry handling |
| [pulse_processor.c](../../src/utils/src/lighthouse/pulse_processor.c) | Common utilities, calibration application |
| [pulse_processor_v1.c](../../src/utils/src/lighthouse/pulse_processor_v1.c) | V1 protocol decoder |
| [pulse_processor_v2.c](../../src/utils/src/lighthouse/pulse_processor_v2.c) | V2 protocol decoder |
| [ootx_decoder.c](../../src/utils/src/lighthouse/ootx_decoder.c) | OOTX bitstream decoder |
| [lighthouse_calibration.c](../../src/utils/src/lighthouse/lighthouse_calibration.c) | Distortion correction |
| [lighthouse_geometry.c](../../src/utils/src/lighthouse/lighthouse_geometry.c) | Ray/intersection math |
| [lighthouse_storage.c](../../src/modules/src/lighthouse/lighthouse_storage.c) | Persistent geo/calib storage |
| [lighthouse_throttle.c](../../src/modules/src/lighthouse/lighthouse_throttle.c) | V2 sample rate limiting |

---

## See Also

- [V1_VS_V2.md](V1_VS_V2.md) - Protocol differences between Lighthouse V1 and V2
- [CALIBRATION.md](CALIBRATION.md) - Detailed calibration model documentation
- [Kalman PIPELINE.md](../kalman/PIPELINE.md) - How measurements are processed by the estimator
