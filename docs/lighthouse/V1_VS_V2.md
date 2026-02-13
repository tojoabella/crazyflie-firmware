# Lighthouse V1 vs V2: Protocol Comparison

This document describes the differences between Lighthouse V1 and V2 protocols as implemented in the Crazyflie firmware.

## Overview

| Feature | V1 (HTC Vive) | V2 (Valve Index) |
|---------|---------------|------------------|
| Max Base Stations | 2 | 16 |
| Rotor Speed | ~60 Hz | ~50-55 Hz (varies by channel) |
| Rotor Tilt | None (0°) | ±30° |
| Sync Method | Omnidirectional sync flash | Implicit (channel encoding) |
| OOTX Source | Sync pulse width | Dedicated slow bit |
| Calibration Terms | 5 (phase, tilt, curve, gib) | 7 (+ogee terms) |

---

## Physical Design

### V1 Base Station

```
┌─────────────────────────────────────┐
│                                     │
│   [LED]                [LED]        │
│                                     │
│   ┌─────┐              ┌─────┐      │
│   │Rotor│              │Rotor│      │
│   │  X  │              │  Y  │      │
│   └─────┘              └─────┘      │
│      │                    │         │
│      v                    v         │
│   Vertical             Horizontal   │
│   Sweep                Sweep        │
│                                     │
│        [Sync LED Flash]             │
│                                     │
└─────────────────────────────────────┘

Sweep Pattern:
  1. Sync flash (omnidirectional IR pulse)
  2. Wait ~20µs (axis-dependent)
  3. Vertical laser sweep (X axis)
  4. Wait ~20µs
  5. Sync flash
  6. Wait ~20µs
  7. Horizontal laser sweep (Y axis)
  8. Repeat at 120 Hz per base station
```

### V2 Base Station

```
┌─────────────────────────────────────┐
│                                     │
│           ┌─────────┐               │
│           │  Single │               │
│           │  Rotor  │               │
│           │         │               │
│           │  ◢ ◣    │  Two lasers   │
│           │ -30 +30 │  tilted ±30°  │
│           └─────────┘               │
│                                     │
│          No Sync LED                │
│                                     │
└─────────────────────────────────────┘

Sweep Pattern:
  - Single rotor with two tilted laser planes
  - Laser 1: -30° tilt
  - Laser 2: +30° tilt
  - Channel encoded in timing/modulation
  - No sync flash needed
  - Rotor speed varies by channel for identification
```

---

## Timing and Frame Structure

### V1 Frame Structure

```
Time (ms)
0.0                                          8.333
│                                              │
│  Sync0  Sync1  X-Sweep     Y-Sweep          │
│    │      │       │           │              │
├────┼──────┼───────┼───────────┼──────────────┤
│ |  │      │       │           │              │
│ |  │      │    BS0 Frame     │              │
│ |  │      │                   │              │
│ |  │      │                   │              │
│    │      │                   │              │
│ Skip │      │  BS1 Frame (offset by 4.166ms) │
│    │                                         │

Total frame: 8.333 ms (120 Hz)
Each base station sweeps at 60 Hz (alternating)
```

**Timing Constants (24 MHz clock)**:
```c
#define FRAME_LENGTH           200000  // 8.333 ms
#define SYNC_BASE_WIDTH        1250    // ~52 µs
#define SYNC_SEPARATION        10000   // ~417 µs between sync0 and sync1
#define SWEEP_CENTER           100000  // Middle of frame
```

### V2 Frame Structure

```
Time (ms)
0.0                              ~39-40
│                                    │
│  ───Continuous rotation───         │
│                                    │
│  Beam1  Beam2  Beam1  Beam2  ...  │
│    │      │      │      │         │
├────┼──────┼──────┼──────┼─────────┤
│ -30°   +30°   -30°   +30°         │
│                                    │

Rotor period: 887,000 - 959,000 ticks (channel-dependent)
Approximately 37-40 ms per rotation
```

**Timing Constants**:
```c
// Rotor periods by channel (24 MHz clock)
static const uint32_t CYCLE_PERIODS[16] = {
    959000, 957000, 953000, 949000,  // Channels 0-3
    947000, 943000, 941000, 939000,  // Channels 4-7
    937000, 929000, 927000, 925000,  // Channels 8-11
    923000, 919000, 917000, 887000   // Channels 12-15
};

#define MAX_TICKS_SENSOR_TO_SENSOR      10000   // Max spread for one sweep
#define MAX_TICKS_BETWEEN_SWEEP_STARTS  10      // Jitter tolerance
```

---

## Synchronization Methods

### V1 Synchronization

V1 uses explicit sync pulses that are omnidirectional (visible from all angles):

```c
typedef enum {
    unknown,
    sync0,   // First sync pulse (from base station 0 or 1)
    sync1,   // Second sync pulse
    sweep    // Actual laser sweep
} pulseClass_e;

// Classification based on pulse width
if (width >= SYNC_MIN_WIDTH && width <= SYNC_MAX_WIDTH) {
    // Sync pulse
    // Determine sync0 vs sync1 based on timing
    if (timeSinceLastSync > SYNC_SEPARATION * 0.8) {
        class = sync0;  // First sync of the frame
    } else {
        class = sync1;  // Second sync (from other BS)
    }
} else if (width < SWEEP_MAX_WIDTH) {
    class = sweep;
}
```

**Synchronization State Machine**:
```
┌─────────────────┐
│ Not Synchronized │
│                 │
│ Collect 8 pulses│
│ per sensor      │
└────────┬────────┘
         │
         │ Pattern match found
         v
┌─────────────────┐
│  Synchronized   │
│                 │
│ Track sync0/sync1
│ timestamps      │
└─────────────────┘
```

### V2 Synchronization

V2 has no sync pulses. Channel identification comes from:
1. LFSR (Linear Feedback Shift Register) data in beam modulation
2. Rotor period (different for each channel)

```c
// Channel is decoded by FPGA from beam modulation
typedef struct {
    // ...
    uint8_t channel;       // 0-15, identifies base station
    bool channelFound;     // True if FPGA decoded channel successfully
    uint8_t slowBit;       // OOTX data bit
} pulseProcessorFrame_t;

// No explicit sync needed - channel field identifies source
if (frame.channelFound) {
    baseStation = frame.channel;
}
```

---

## Angle Calculation

### V1 Angle Calculation

```c
// Angle from sweep timing relative to sync
float calculateAngleV1(uint32_t sweepTs, uint32_t syncTs, float frameWidth) {
    // Time from sync to sweep detection
    uint32_t delta = TS_DIFF(sweepTs, syncTs);

    // Normalize to [-0.5, +0.5] of frame
    float normalized = (float)delta / frameWidth - 0.5f;

    // Convert to angle (radians)
    // Full sweep is approximately ±60° = ±π/3
    float angle = normalized * 2.0f * M_PI / 3.0f;

    return angle;
}
```

**Angular Range**: Approximately ±60° (±1.047 radians)

### V2 Angle Calculation

```c
// Angle from rotor offset
float calculateAngleV2(uint32_t offset, uint8_t channel, int beamIndex) {
    // Get period for this channel
    float period = CYCLE_PERIODS[channel];

    // Convert offset to radians
    float baseAngle = (float)offset * 2.0f * M_PI_F / period;

    // Adjust for beam tilt (±30°)
    float tiltOffset = (beamIndex == 0) ? M_PI_F / 3.0f : -M_PI_F / 3.0f;

    // Final angle (subtract π to center at 0)
    float angle = baseAngle - M_PI_F + tiltOffset;

    return angle;
}
```

**Angular Range**: Approximately ±60° per beam, but beams are tilted

### V2 to V1 Angle Conversion

For compatibility, V2 angles can be converted to V1-equivalent format:

```c
void pulseProcessorV2ConvertToV1Angles(float v2Angles[2], float v1Angles[2]) {
    // V2: beam1 at -30°, beam2 at +30°
    // V1: horizontal and vertical sweeps

    const float tan30 = 0.5773502691896258f;

    // Convert tilted beam angles to horizontal/vertical
    float a1 = v2Angles[0];
    float a2 = v2Angles[1];

    // Effective horizontal angle
    v1Angles[0] = (a1 + a2) / 2.0f;

    // Effective vertical angle
    v1Angles[1] = (a2 - a1) / (2.0f * tan30);
}
```

---

## OOTX Data Transmission

### V1 OOTX

```c
// Bit encoded in sync pulse width
// Bit 1 position in width offset
uint16_t widthOffset = syncPulseWidth - SYNC_BASE_WIDTH;
uint8_t ootxBit = (widthOffset >> 1) & 0x01;

// Other bits encode axis and skip information:
// Bit 0: Skip indicator
// Bit 1: OOTX data
// Bit 2: Axis (0=X, 1=Y)
```

**Rate**: ~120 bits/second (one bit per sync pulse)

### V2 OOTX

```c
// Bit available directly in frame
uint8_t ootxBit = frame.slowBit;

// Transmitted sparsely (not every sweep)
// Rate limited by MIN_TICKS_BETWEEN_SLOW_BITS
```

**Rate**: ~1.4 bits/second per channel (much slower than V1)

---

## Calibration Differences

### V1 Calibration Model

```c
typedef struct {
    float phase;      // Phase offset
    float tilt;       // Plane tilt
    float curve;      // Curvature
    float gibmag;     // 2nd harmonic magnitude
    float gibphase;   // 2nd harmonic phase
} lighthouseCalibrationSweep_t;

// Applied without rotor tilt consideration
float t = 0.0f;  // No tilt for V1
```

### V2 Calibration Model

```c
typedef struct {
    float phase;
    float tilt;
    float curve;
    float gibmag;
    float gibphase;
    float ogeemag;    // 3rd harmonic magnitude (V2 only)
    float ogeephase;  // 3rd harmonic phase (V2 only)
} lighthouseCalibrationSweep_t;

// Applied with rotor tilt
float t = (sweep == 0) ? -M_PI_F / 6.0f : M_PI_F / 6.0f;  // ±30°
```

---

## Data Structures

### V1-Specific State

```c
typedef struct {
    bool synchronized;
    int baseStationsSynchronizedCount;

    // Pulse history for sync detection
    pulseProcessorPulse_t pulseHistory[4][8];  // 4 sensors, 8 history depth
    int pulseHistoryIdx[4];

    // Sync timing
    uint32_t lastSync;
    uint32_t currentSync0, currentSync1;
    uint32_t currentSync0Width, currentSync1Width;
    float frameWidth[2][2];  // [base_station][axis]

    // Current frame info
    int currentBaseStation;
    SweepId_t currentAxis;

    // Sweep data
    struct {
        uint32_t timestamp;
        SweepStorageState_t state;
    } sweeps[4];
    bool sweepDataStored;
} pulseProcessorV1_t;
```

### V2-Specific State

```c
typedef struct {
    pulseProcessorV2PulseWorkspace_t pulseWorkspace;   // Pulse accumulator
    pulseProcessorV2BlockWorkspace_t blockWorkspace;   // Block decoder

    // Latest block per channel for pairing
    pulseProcessorV2SweepBlock_t blocks[CONFIG_DECK_LIGHTHOUSE_MAX_N_BS];

    // OOTX timestamps for rate limiting
    uint32_t ootxTimestamps[CONFIG_DECK_LIGHTHOUSE_MAX_N_BS];
} pulseProcessorV2_t;

typedef struct {
    uint32_t offset[4];    // Rotor offset per sensor
    uint32_t timestamp0;   // Rotor zero position timestamp
    uint8_t channel;       // Base station channel
} pulseProcessorV2SweepBlock_t;
```

---

## Multi-Base Station Support

### V1: Two Base Stations

- Stations alternate: BS0 sweeps, then BS1 sweeps
- Sync pulses distinguish between stations
- Time-division multiplexing

```
Time →
BS0: |--sweep--|  ·········  |--sweep--|  ·········
BS1:  ·········  |--sweep--|  ·········  |--sweep--|
      └──4.166ms──┘          └──4.166ms──┘
```

### V2: Up to 16 Base Stations

- Each station uses a unique channel (0-15)
- Channels identified by rotor period and LFSR pattern
- Frequency-division multiplexing (different speeds)
- All stations can operate simultaneously

```
BS0 (ch0):  ────────────────────────────────────────
BS1 (ch1):  ────────────────────────────────────────
BS2 (ch2):  ────────────────────────────────────────
...
BS15 (ch15): ───────────────────────────────────────
              All operating simultaneously
```

---

## Measurement Quality

### V1 Characteristics

| Property | Value |
|----------|-------|
| Update rate | ~60 Hz per base station |
| Angular precision | ~0.1° |
| Sync robustness | Requires clear LOS to sync LED |
| Multi-room | Difficult (sync interference) |

### V2 Characteristics

| Property | Value |
|----------|-------|
| Update rate | ~50-55 Hz per base station |
| Angular precision | ~0.05° |
| Sync robustness | No sync LED (implicit) |
| Multi-room | Supported (16 channels) |

---

## Implementation Switching

The firmware automatically detects and switches between V1 and V2:

```c
// Function pointer for protocol-specific processing
pulseProcessorProcessPulse_t pulseProcessorProcessPulse;

// Set based on detected system type
void lighthouseCoreSetSystemType(lighthouseBaseStationType_t type) {
    systemType = type;
    if (type == lighthouseBsTypeV1) {
        pulseProcessorProcessPulse = pulseProcessorV1ProcessPulse;
    } else {
        pulseProcessorProcessPulse = pulseProcessorV2ProcessPulse;
    }
}

// Detection based on received frames
if (frame.channelFound) {
    // V2 frame (has channel info)
    lighthouseCoreSetSystemType(lighthouseBsTypeV2);
} else if (classifyPulse(frame) != unknown) {
    // V1 frame (sync pulse detected)
    lighthouseCoreSetSystemType(lighthouseBsTypeV1);
}
```

---

## File Reference

| File | V1 Support | V2 Support |
|------|------------|------------|
| [pulse_processor_v1.c](../../src/utils/src/lighthouse/pulse_processor_v1.c) | ✓ | - |
| [pulse_processor_v2.c](../../src/utils/src/lighthouse/pulse_processor_v2.c) | - | ✓ |
| [pulse_processor.c](../../src/utils/src/lighthouse/pulse_processor.c) | ✓ | ✓ |
| [lighthouse_calibration.c](../../src/utils/src/lighthouse/lighthouse_calibration.c) | ✓ | ✓ |
| [lighthouse_core.c](../../src/modules/src/lighthouse/lighthouse_core.c) | ✓ | ✓ |

---

## See Also

- [PIPELINE.md](PIPELINE.md) - Complete data flow
- [CALIBRATION.md](CALIBRATION.md) - Calibration system details
