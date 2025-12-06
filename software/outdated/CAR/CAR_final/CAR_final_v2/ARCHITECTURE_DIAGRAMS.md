# Architecture Diagrams

## System Overview

```
┌───────────────────────────────────────────────────────────────────────┐
│                        RP2040 Microcontroller                         │
│                                                                       │
│  ┌────────────────────────────┐  ┌────────────────────────────────┐ │
│  │         CORE 0             │  │          CORE 1               │ │
│  │    State Machine           │  │    I/O Scheduler              │ │
│  │    (Pure Logic)            │  │    (Hardware)                 │ │
│  │                            │  │                               │ │
│  │  ┌──────────────────────┐ │  │  ┌─────────────────────────┐ │ │
│  │  │ Read Sensor Data     │ │  │  │ Time Slot 0: IR Read    │ │ │
│  │  │ (from shared memory) │ │  │  │ Time Slot 1: ToF 0      │ │ │
│  │  └──────────┬───────────┘ │  │  │ Time Slot 2: IR Read    │ │ │
│  │             │              │  │  │ Time Slot 3: ToF 1      │ │ │
│  │             ▼              │  │  │ Time Slot 4: IR+Button  │ │ │
│  │  ┌──────────────────────┐ │  │  │ Time Slot 5: ToF 2+Btn  │ │ │
│  │  │ Edge Detection       │ │  │  │ Time Slot 6: IR Read    │ │ │
│  │  │ (Highest Priority)   │ │  │  │ Time Slot 7: ToF 3+Mtr  │ │ │
│  │  └──────────┬───────────┘ │  │  │ Time Slot 8: WiFi       │ │ │
│  │             │              │  │  │ Time Slot 9: ToF 4      │ │ │
│  │             ▼              │  │  └─────────────────────────┘ │ │
│  │  ┌──────────────────────┐ │  │                               │ │
│  │  │ Target Detection     │ │  │  Each slot: 50ms              │ │
│  │  │ & Tracking Logic     │ │  │  Total cycle: 500ms           │ │
│  │  └──────────┬───────────┘ │  │                               │ │
│  │             │              │  │  ┌─────────────────────────┐ │ │
│  │             ▼              │  │  │ Write Sensor Data       │ │ │
│  │  ┌──────────────────────┐ │  │  │ (to shared memory)      │ │ │
│  │  │ State Transitions    │ │  │  └─────────────────────────┘ │ │
│  │  │ (6 states)           │ │  │                               │ │
│  │  └──────────┬───────────┘ │  │  ┌─────────────────────────┐ │ │
│  │             │              │  │  │ Read Motor Commands     │ │ │
│  │             ▼              │  │  │ (from shared memory)    │ │ │
│  │  ┌──────────────────────┐ │  │  └───────────┬─────────────┘ │ │
│  │  │ Write Motor Commands │ │  │              │               │ │
│  │  │ (to shared memory)   │ │  │              ▼               │ │
│  │  └──────────────────────┘ │  │  ┌─────────────────────────┐ │ │
│  │                            │  │  │ Execute Motor PWM       │ │ │
│  │  Loop: 100Hz (10ms)       │  │  └─────────────────────────┘ │ │
│  └────────────────────────────┘  └────────────────────────────────┘ │
│                                                                       │
│         ▲                                          ▲                  │
│         │          Mutex-Protected                │                  │
│         │          Shared Memory                  │                  │
│         │                                          │                  │
│  ┌──────┴────────────────────────────────────────┴──────┐           │
│  │  SensorData    MotorCommand    TelemetryData         │           │
│  └───────────────────────────────────────────────────────┘           │
└───────────────────────────────────────────────────────────────────────┘
```

## Core 0 State Machine Flow

```
┌─────────┐
│  IDLE   │ ◄────────────────────────┐
└────┬────┘                           │
     │ Button Press                   │
     ▼                                │
┌─────────────┐                       │
│  SEARCHING  │ ◄────────┐            │
└──────┬──────┘          │            │
       │ Target          │ Lost       │
       │ Detected        │ Long       │
       ▼                 │            │
┌─────────────┐          │            │
│  TRACKING   │──────────┘            │
└──────┬──────┘                       │
       │ Centered                     │
       ▼                              │
┌─────────────┐                       │
│  ATTACKING  │                       │
└──────┬──────┘                       │
       │ Lost                         │
       │ Alignment                    │
       └──────────────────────────────┘
       
       Edge Detected (Any State)
              │
              ▼
       ┌─────────────────┐
       │ EDGE_AVOIDING   │
       └────────┬─────────┘
                │ Escape Complete
                └────────────────────> SEARCHING
```

## Core 1 Time Slot Execution

```
Time:  0ms    50ms   100ms  150ms  200ms  250ms  300ms  350ms  400ms  450ms  500ms
       │      │      │      │      │      │      │      │      │      │      │
Slot:  0      1      2      3      4      5      6      7      8      9      0...
       │      │      │      │      │      │      │      │      │      │      │
       ▼      ▼      ▼      ▼      ▼      ▼      ▼      ▼      ▼      ▼      ▼
      IR     ToF0    IR    ToF1   IR+    ToF2+   IR    ToF3+  WiFi   ToF4    IR...
      5ms    35ms    5ms   35ms   Btn    Btn     5ms   Mtr    10ms   35ms
                                   10ms   40ms          36ms

IR Sensors:    ●─────────────●─────────────●─────────────●───────────── (200Hz avg)
ToF Sensors:   ──●────────────────●────────────────●─────────────────── (2Hz each)
Buttons:       ──────────────────────────●─────●──────────────────────── (4Hz)
Motors:        ────────────────────────────────────────●─────────────── (2Hz)
WiFi:          ──────────────────────────────────────────────●───────── (2Hz)
```

## Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                         Hardware Layer                            │
└──────────────────────────────────────────────────────────────────┘
    │                    │                    │               │
    │ IR Sensors         │ ToF Sensors        │ Buttons       │ Motors
    │ (ADS1115)          │ (VL53L0X)          │               │
    ▼                    ▼                    ▼               ▲
┌────────────────────────────────────────────────────────────┴──────┐
│                        Core 1: I/O Layer                          │
│                                                                   │
│  Ads1115Sampler      ToFArray           ButtonManager    Car     │
│  └─► readAll()       └─► readOne()     └─► sample()     └─► set  │
└───────────────────────────────────────────┬───────────────────────┘
                                            │
                                            ▼
┌───────────────────────────────────────────────────────────────────┐
│                    Shared Memory (Mutex Protected)                │
│                                                                   │
│  SensorData {                   MotorCommand {                   │
│    irVoltages[4]                  leftSpeed                      │
│    tofDistances[5]                rightSpeed                     │
│    tofValid[5]                    emergencyStop                  │
│    buttonMode                     timestamp                      │
│    timestamp                    }                                │
│  }                                                               │
└───────────────────────────────────────┬───────────────────────────┘
                                        │
                                        ▼
┌───────────────────────────────────────────────────────────────────┐
│                     Core 0: Logic Layer                           │
│                                                                   │
│  State Machine                                                    │
│  ├─► detectEdge(sensors) → bool                                 │
│  ├─► detectTarget(sensors) → TargetInfo                         │
│  ├─► transitionState(newState)                                  │
│  └─► executeState()                                             │
│                                                                   │
│  Decision Functions                                               │
│  ├─► calculateMotorSpeeds(state, target)                        │
│  ├─► planEscapeManeuver(edgePattern)                            │
│  └─► selectSearchPattern()                                      │
└───────────────────────────────────────────────────────────────────┘
```

## Memory Layout

```
┌─────────────────────────────────────────────────────────────┐
│                    RP2040 Memory Space                      │
├─────────────────────────────────────────────────────────────┤
│  Flash (2MB)                                                │
│  ├─ Program Code (~100KB)                                   │
│  ├─ String Constants (~10KB)                                │
│  └─ Available (~1.9MB)                                      │
├─────────────────────────────────────────────────────────────┤
│  SRAM (264KB)                                               │
│  ├─ Core 0 Stack (~4KB)                                     │
│  ├─ Core 1 Stack (~4KB)                                     │
│  ├─ Heap (~200KB)                                           │
│  │   ├─ WiFi Buffers (~20KB)                                │
│  │   ├─ Sensor Buffers (~5KB)                               │
│  │   ├─ String Buffers (~10KB)                              │
│  │   └─ Available (~165KB)                                  │
│  ├─ Shared Memory (~1KB)                                    │
│  │   ├─ SensorData (128 bytes)                              │
│  │   ├─ MotorCommand (32 bytes)                             │
│  │   ├─ TelemetryData (256 bytes)                           │
│  │   └─ Mutexes (48 bytes)                                  │
│  └─ Global Variables (~55KB)                                │
│      ├─ State Machine (~1KB)                                │
│      ├─ WiFi/TCP (~40KB)                                    │
│      ├─ Sensor Objects (~10KB)                              │
│      └─ Motor Objects (~4KB)                                │
└─────────────────────────────────────────────────────────────┘
```

## Timing Analysis

```
Core 0 Execution Timeline (10ms cycle):
┌──────────────────────────────────────────────────────────┐
│ [0.00ms] readSensorData() ────────────────── [0.01ms]    │
│ [0.01ms] detectEdge() ──────────────────────── [0.05ms]  │
│ [0.05ms] detectTarget() ────────────────────── [0.08ms]  │
│ [0.08ms] stateMachine.execute() ───────────── [0.10ms]   │
│ [0.10ms] writeMotorCommand() ──────────────── [0.11ms]   │
│ [0.11ms] updateTelemetry() ────────────────── [0.12ms]   │
│ [0.12ms] delay(9.88ms) ─────────────────────── [10.00ms] │
└──────────────────────────────────────────────────────────┘
Total CPU: ~1.2% (0.12ms / 10ms)

Core 1 Execution Timeline (500ms cycle):
┌──────────────────────────────────────────────────────────┐
│ Slot 0 [0ms]:    IR Read ──────────────────── 5ms        │
│ Slot 1 [50ms]:   ToF 0 Read ──────────────── 35ms        │
│ Slot 2 [100ms]:  IR Read ──────────────────── 5ms        │
│ Slot 3 [150ms]:  ToF 1 Read ──────────────── 35ms        │
│ Slot 4 [200ms]:  IR+Button ─────────────────── 10ms      │
│ Slot 5 [250ms]:  ToF 2+Debounce ───────────── 40ms       │
│ Slot 6 [300ms]:  IR Read ──────────────────── 5ms        │
│ Slot 7 [350ms]:  ToF 3+Motor ──────────────── 36ms       │
│ Slot 8 [400ms]:  WiFi ─────────────────────── 10ms       │
│ Slot 9 [450ms]:  ToF 4 ────────────────────── 35ms       │
└──────────────────────────────────────────────────────────┘
Total CPU: ~38.2% (191ms / 500ms)
```

## State Machine Detail

```
IDLE State:
  Inputs: buttonMode
  Outputs: motors = 0
  Transitions:
    → SEARCHING (if buttonMode == RUN)

SEARCHING State:
  Inputs: tofSensors, irSensors
  Outputs: motors = spin
  Logic:
    1. Check edge → EDGE_AVOIDING
    2. Check target → TRACKING
    3. Else: continue spin
  Transitions:
    → EDGE_AVOIDING (if edge detected)
    → TRACKING (if target found)

TRACKING State:
  Inputs: tofSensors, irSensors
  Outputs: motors = align
  Logic:
    1. Check edge → EDGE_AVOIDING
    2. Calculate bias
    3. If centered → ATTACKING
    4. Else: align motors
  Transitions:
    → EDGE_AVOIDING (if edge detected)
    → ATTACKING (if bias < deadzone)
    → SEARCHING (if lost > 2s)

ATTACKING State:
  Inputs: tofSensors, irSensors
  Outputs: motors = full forward
  Logic:
    1. Check edge → EDGE_AVOIDING
    2. Verify centered
    3. If not → TRACKING
  Transitions:
    → EDGE_AVOIDING (if edge detected)
    → TRACKING (if not centered)

EDGE_AVOIDING State:
  Inputs: irSensors, elapsed time
  Outputs: motors = escape pattern
  Logic:
    Multi-stage escape based on pattern:
    0-500ms: Back
    500-1500ms: Turn
    >1500ms: Complete
  Transitions:
    → SEARCHING (after escape complete)
```

## Sensor Placement

```
Top View:
           ┌─────────────┐
           │   ToF L45   │ Pin 4
           │             │
    ToF L23│    ROBOT    │ToF R23
    Pin 5  │             │Pin 7
           │   ToF M0    │ Pin 6
           │             │
           │   ToF R45   │ Pin 8
           └─────────────┘

IR Sensor Layout:
           ┌─────────────┐
      A1   │             │ A2
    Front-L│             │Front-R
           │             │
           │             │
      A0   │             │ A3
     Back-L│             │Back-R
           └─────────────┘
```

---

All diagrams are ASCII-art for easy viewing in any text editor!
