# RP2040 Dual-Core Bottle Sumo Firmware Architecture

**Version:** 1.0.0  
**Target Platform:** Raspberry Pi Pico W (RP2040 dual-core ARM Cortex-M0+)  
**Framework:** arduino-pico (Earle F. Philhower III)  
**Date:** 2025-10-06

---

## 📋 Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Core Assignment & Responsibilities](#core-assignment--responsibilities)
3. [Lockless Communication Protocol](#lockless-communication-protocol)
4. [Timing Specifications & WCET Analysis](#timing-specifications--wcet-analysis)
5. [Hardware Configuration](#hardware-configuration)
6. [Module Descriptions](#module-descriptions)
7. [Tuning Parameters](#tuning-parameters)
8. [Memory Footprint Analysis](#memory-footprint-analysis)
9. [Compilation & Upload Instructions](#compilation--upload-instructions)
10. [Troubleshooting & Debugging](#troubleshooting--debugging)

---

## Architecture Overview

This firmware implements a **lockless dual-core architecture** for high-performance sensor fusion and real-time tactical decision-making in a Bottle Sumo robot competition environment.

### Key Design Principles

1. **Deterministic Timing:** All tasks use millis()-based scheduling with fixed periods (no delay() in hot paths)
2. **Zero Dynamic Allocation:** All memory statically allocated (no malloc/free during runtime)
3. **Lockless Core Communication:** Double-buffered snapshots with memory barriers eliminate blocking
4. **Modular Design:** Clear separation of acquisition (Core1) and decision-making (Core0)

### System Block Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         RP2040 Dual-Core                         │
├─────────────────────────────┬───────────────────────────────────┤
│         CORE 1               │           CORE 0                  │
│      (Acquisition)           │       (State Machine)             │
├─────────────────────────────┼───────────────────────────────────┤
│                              │                                   │
│  ┌────────────────────────┐ │  ┌─────────────────────────────┐ │
│  │  Task Scheduler        │ │  │   Snapshot Consumer         │ │
│  │  (millis-based)        │ │  │   - Fetch latest snapshot   │ │
│  └────────────────────────┘ │  │   - Staleness check         │ │
│            │                 │  └─────────────────────────────┘ │
│            ▼                 │              │                    │
│  ┌────────────────────────┐ │              ▼                    │
│  │  IR Acquisition (10ms) │ │  ┌─────────────────────────────┐ │
│  │  - ADS1115 read        │ │  │   State Machine Logic       │ │
│  │  - Edge detection      │ │  │   - Edge avoidance          │ │
│  │  - Danger level        │ │  │   - Opponent attack         │ │
│  └────────────────────────┘ │  │   - Search pattern          │ │
│            │                 │  └─────────────────────────────┘ │
│            ▼                 │              │                    │
│  ┌────────────────────────┐ │              ▼                    │
│  │  ToF Poll (20ms/sensor)│ │  ┌─────────────────────────────┐ │
│  │  - Continuous mode     │ │  │   Command Generator         │ │
│  │  - Staggered (0/7/14ms│ │  │   - Motor PWM               │ │
│  │  - Opponent detection  │ │  │   - Threshold updates       │ │
│  └────────────────────────┘ │  └─────────────────────────────┘ │
│            │                 │              │                    │
│            ▼                 │              ▼                    │
│  ┌────────────────────────┐ │  ┌─────────────────────────────┐ │
│  │  Button Debounce (20ms)│ │  │   CommandBlock Writer       │ │
│  │  - 5ms sampling        │ │  │   - Seq increment           │ │
│  │  - 4-sample history    │ │  │   - Memory barrier          │ │
│  └────────────────────────┘ │  └─────────────────────────────┘ │
│            │                 │              │                    │
│            ▼                 │              ▼                    │
│  ┌────────────────────────┐ │  ┌─────────────────────────────┐ │
│  │  Snapshot Publisher    │◀├──┤   WiFi/TCP Manager (stub)  │ │
│  │  - Double buffer       │ │  │   - Client connections      │ │
│  │  - Memory barrier      │ │  │   - Telemetry streaming     │ │
│  │  - Sequence increment  │ │  └─────────────────────────────┘ │
│  └────────────────────────┘ │                                   │
│            │                 │                                   │
│            ▼                 │                                   │
│  ┌────────────────────────┐ │                                   │
│  │  Telemetry Builder     │ │                                   │
│  │  - JSON (50ms / 20Hz)  │ │                                   │
│  │  - Static buffer       │ │                                   │
│  └────────────────────────┘ │                                   │
│            │                 │                                   │
│            ▼                 │                                   │
│  ┌────────────────────────┐ │                                   │
│  │  Command Poller        │◀├──────────────────────────────────┤
│  │  - Check seq change    │ │          (CommandBlock)           │
│  │  - Apply thresholds    │ │                                   │
│  │  - Apply motor cmds    │ │                                   │
│  └────────────────────────┘ │                                   │
│                              │                                   │
└─────────────────────────────┴───────────────────────────────────┘
```

---

## Core Assignment & Responsibilities

### Core 1 (Acquisition Core)

**Primary Responsibilities:**
- High-frequency sensor polling (IR, ToF, buttons)
- Data processing and metric derivation
- Lockless snapshot publishing to Core0
- JSON telemetry generation (20 Hz)
- Command reception from Core0

**Loop Frequency:** ~500 Hz (limited by scheduler task density, not explicit delay)

**Key Tasks (Scheduled):**
| Task | Period | Frequency | WCET | Notes |
|------|--------|-----------|------|-------|
| IR Acquisition | 10 ms | 100 Hz | 1.5 ms | ADS1115 read + edge detection |
| Button Sampling | 5 ms | 200 Hz | <50 µs | Raw pin reads (stub) |
| Button Debounce | 20 ms | 50 Hz | <100 µs | 4-sample history check |
| ToF Poll (RIGHT) | 20 ms | 50 Hz | 1-2 ms | Non-blocking if data ready |
| ToF Poll (FRONT) | 20 ms | 50 Hz | 1-2 ms | Staggered +7ms offset |
| ToF Poll (LEFT) | 20 ms | 50 Hz | 1-2 ms | Staggered +14ms offset |
| Snapshot Publish | 10 ms | 100 Hz | <10 µs | Double-buffer flip + DMB |
| Telemetry JSON | 50 ms | 20 Hz | ~5 ms | snprintf string formatting |
| Command Poll | Every loop | ~500 Hz | <50 µs | Check seq, apply commands |
| Statistics | 1000 ms | 1 Hz | <100 µs | Serial print diagnostics |

**Total CPU Load (Worst Case):**
```
IR:        1.5ms/10ms  = 15%
ToF (3x):  6ms/20ms    = 30% (staggered, actual ~10% per sensor)
Telemetry: 5ms/50ms    = 10%
Overhead:               ~5%
-----------------------------------------
Total:                  ~60% (typical)
                        ~80% (peak with all ToF data ready simultaneously)
```

### Core 0 (State Machine Core)

**Primary Responsibilities:**
- Fetch latest snapshots from Core1
- Tactical decision-making (state machine)
- Motor command generation
- WiFi AP and TCP server management (stub)
- OLED display updates (stub, uses Wire1 mutex)
- Threshold configuration via TCP commands

**Loop Frequency:** ~100 Hz (10 ms delay per loop)

**Key Operations:**
| Operation | Frequency | WCET | Notes |
|-----------|-----------|------|-------|
| Snapshot Fetch | Every loop (~100 Hz) | <50 µs | Lockless read with retry |
| Staleness Check | Every loop | <10 µs | Compare timestamps |
| State Machine | Every loop | <100 µs | Tactical logic (stub) |
| Command Update | As needed | <10 µs | Write CommandBlock + DMB |
| WiFi/TCP Handler | 25-50 ms throttled | TBD | Stub (future implementation) |
| OLED Update | TBD (future) | ~20 ms | Wire1 mutex required |

**Total CPU Load (Estimated):**
```
Snapshot Fetch:  0.05ms/10ms = 0.5%
State Machine:   0.1ms/10ms  = 1%
Overhead:                     ~5%
WiFi/TCP (future):            ~10-20%
-----------------------------------------
Total (current):              ~7% (mostly idle)
Total (with WiFi):            ~20-30% (future)
```

---

## Lockless Communication Protocol

### Core1 → Core0: Sensor Snapshots (Double-Buffered)

**Data Structure:** `SensorSnapshotExchange`
- 2× `SensorSnapshot` buffers (~256 bytes total)
- `volatile uint8_t publishIndex` (0 or 1)
- `volatile uint32_t latestSequence`

**Write Protocol (Core1):**
```cpp
1. Determine inactive buffer: inactiveIdx = 1 - publishIndex
2. Copy draft snapshot to buffers[inactiveIdx]
3. Memory barrier (DMB) - ensure writes complete
4. Atomically flip: publishIndex = inactiveIdx
5. Update latestSequence = draft.sequence
6. Memory barrier (DMB) - ensure visibility
```

**Read Protocol (Core0):**
```cpp
1. Read current publishIndex → startIdx
2. Memory barrier (DMB) - ensure fresh read
3. Copy buffers[startIdx] to output
4. Memory barrier (DMB) - ensure copy complete
5. Re-read publishIndex → endIdx
6. If startIdx == endIdx: SUCCESS
7. Else: RETRY once (race condition during copy)
```

**Race Condition Handling:**
- If Core1 flips `publishIndex` mid-copy, Core0 detects mismatch and retries
- Retry succeeds with high probability (Core1 flip rate = 100 Hz, copy time <20 µs → collision probability <0.2%)
- After 2 failed attempts, Core0 continues with stale data (extremely rare)

**Memory Ordering:**
- ARM Cortex-M0+ uses relaxed memory model
- `__DMB()` (Data Memory Barrier) ensures all writes visible to other core before proceeding
- Critical for preventing Core0 from reading partially updated snapshot

### Core0 → Core1: Commands (Sequence-Based)

**Data Structure:** `CommandBlock`
- `volatile uint32_t seq` (command sequence number)
- Motor commands: `int16_t motorLeft, motorRight`
- Thresholds: `float thresholds[4], uint8_t thresholdMask`
- Flags: `uint16_t flags` (future expansion)

**Write Protocol (Core0):**
```cpp
1. Memory barrier (DMB)
2. Write motorLeft, motorRight
3. Write thresholds[] and thresholdMask (if updating)
4. Increment seq
5. Memory barrier (DMB)
```

**Read Protocol (Core1):**
```cpp
1. Memory barrier (DMB)
2. Read seq
3. If seq == lastSeq: RETURN (no new commands)
4. Read motorLeft, motorRight, thresholds[], mask
5. Apply commands (validate thresholds first)
6. Update lastSeq = seq
7. Memory barrier (DMB)
```

**Advantages Over Mutex:**
- Core1 never blocks (worst case: uses old commands for 1 cycle)
- Core0 writes are instantaneous (<10 µs vs. ~20 µs mutex overhead)
- No priority inversion or deadlock risk

---

## Timing Specifications & WCET Analysis

### Core1 Task Timing Diagram (First 100ms)

```
Time (ms):  0    10   20   30   40   50   60   70   80   90   100
            │────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼
IR:         █    █    █    █    █    █    █    █    █    █    
ToF-R:      ░    ░░   ░    ░░   ░    ░░   ░    ░░   ░    ░░   
ToF-F:         ░░   ░░   ░░   ░░   ░░   ░░   ░░   ░░   ░░  
ToF-L:            ░░   ░░   ░░   ░░   ░░   ░░   ░░   ░░   ░░
Snapshot:   ▲    ▲    ▲    ▲    ▲    ▲    ▲    ▲    ▲    ▲    
Telemetry:  ▼              ▼              ▼              ▼

Legend:
█ = IR acquisition (1.5ms)
░ = ToF poll (1-2ms if data ready, <0.1ms if not ready)
▲ = Snapshot publish (<10µs)
▼ = Telemetry JSON build (5ms)
```

**Key Observations:**
1. **ToF Staggering:** Sensors polled 7ms apart prevents I2C bus congestion
2. **IR Dominance:** IR task consumes most CPU (15% duty cycle)
3. **Telemetry Spikes:** Every 50ms, adds 5ms CPU burst (10% duty cycle)
4. **Snapshot Overhead:** Negligible (<0.1% duty cycle)

### Worst-Case Execution Time (WCET) Breakdown

**Core1 Single Loop Iteration (Worst Case):**
```
Task                  | WCET     | Notes
----------------------|----------|----------------------------------------
Command Poll          | 50 µs    | Check seq + apply (if changed: 200 µs)
IR Acquisition        | 1500 µs  | 4× ADS1115 reads + processing
Button Sample         | 50 µs    | Raw pin reads
Button Debounce       | 100 µs   | History comparison
ToF Poll RIGHT        | 2000 µs  | If data ready (else 100 µs)
ToF Poll FRONT        | 2000 µs  | If data ready
ToF Poll LEFT         | 2000 µs  | If data ready
Snapshot Publish      | 10 µs    | Memory copy + DMB
Telemetry JSON        | 5000 µs  | snprintf formatting
Statistics            | 100 µs   | Serial print
----------------------|----------|----------------------------------------
Total (all tasks)     | 12810 µs | ~12.8 ms

Loop period: NOT FIXED (scheduler-driven, tasks only run when due)
Actual loop frequency: ~500 Hz (2 ms average iteration when no tasks due)
```

**Peak CPU Scenario (All Tasks Due Simultaneously):**
- Occurs once every 1000 ms (LCM of all task periods)
- Duration: ~13 ms
- CPU idle remaining: 1000 - 13 = 987 ms
- Peak load: 13/1000 = 1.3% (trivial)

**Typical CPU Scenario (10ms Period):**
- IR (1.5ms) + Snapshot (0.01ms) + Command (0.05ms) = 1.56 ms
- CPU idle: 10 - 1.56 = 8.44 ms
- Typical load: 15.6%

---

## Hardware Configuration

### I2C Bus Allocation

**Wire (Default Pins - typically GP4/GP5 on Pico W):**
- **ADS1115 ADC:** Address 0x48
  - Channel 0: Front-Left IR sensor (QRE1113)
  - Channel 1: Front-Right IR sensor
  - Channel 2: Back-Left IR sensor
  - Channel 3: Back-Right IR sensor
  - Configuration: GAIN_ONE (±4.096V), 860 SPS
  - Clock: 400 kHz (fast mode)

**Wire1 (GP26 SDA / GP27 SCL):**
- **VL53L0X ToF Sensors:**
  - 0x30: RIGHT sensor (XSHUT on GP11)
  - 0x31: FRONT sensor (XSHUT on GP12)
  - 0x32: LEFT sensor (XSHUT on GP13)
  - Configuration: 25ms timing budget, continuous mode
  - Clock: 400 kHz (fast mode)
  
- **SSD1306 OLED Display (Future):**
  - Address: 0x3C
  - Size: 128×64 pixels
  - **Mutex Required:** g_wire1Mutex protects shared bus access

### GPIO Pin Assignments

| Pin | Function | Direction | Notes |
|-----|----------|-----------|-------|
| GP4 | I2C0 SDA (Wire) | I/O | ADS1115 |
| GP5 | I2C0 SCL (Wire) | Output | ADS1115 |
| GP11 | ToF XSHUT RIGHT | Output | Pull low to reset sensor |
| GP12 | ToF XSHUT FRONT | Output | Pull low to reset sensor |
| GP13 | ToF XSHUT LEFT | Output | Pull low to reset sensor |
| GP14 | Button START (stub) | Input | Active-low with pullup |
| GP15 | Button STOP (stub) | Input | Active-low with pullup |
| GP26 | I2C1 SDA (Wire1) | I/O | ToF + OLED |
| GP27 | I2C1 SCL (Wire1) | Output | ToF + OLED |

**Motor Driver Pins (Future Expansion):**
- TBD - depends on driver IC (L298N, DRV8833, etc.)
- Typical: 4× PWM pins for dual H-bridge control

### Power Requirements

| Component | Voltage | Current (Peak) | Notes |
|-----------|---------|----------------|-------|
| RP2040 (Pico W) | 3.3V | ~150 mA | Core + WiFi radio |
| ADS1115 | 3.3V | ~1 mA | Very low power |
| VL53L0X (×3) | 3.3V | ~60 mA | 20 mA each, continuous mode |
| OLED SSD1306 | 3.3V | ~20 mA | Peak during full white screen |
| **Total (no motors)** | 3.3V | **~230 mA** | Pico W 3V3 OUT can supply 300 mA |

**Critical:** Motors require separate power supply (7.2-12V LiPo) with common ground to Pico.

---

## Module Descriptions

### Module A: ToF Continuous Mode

**File Section:** Lines 565-681 in `BottleSumo_RP2040_DualCore.ino`

**Key Functions:**
- `initToFSensors()`: Sequential init with XSHUT control
- `pollOneToFSensor(idx, snapshot)`: Non-blocking poll

**Initialization Sequence:**
1. Pull all XSHUT pins LOW (reset all sensors)
2. Wait 10ms
3. Release RIGHT sensor XSHUT (HIGH) → wake, assign 0x30
4. Configure timing budget (25ms), start continuous mode
5. Wait 7ms (stagger)
6. Release FRONT sensor XSHUT → wake, assign 0x31, configure, start
7. Wait 7ms (stagger)
8. Release LEFT sensor XSHUT → wake, assign 0x32, configure, start

**Continuous Mode Operation:**
- Sensors autonomously take measurements every ~25ms
- `pollOneToFSensor()` checks `isRangeComplete()` (non-blocking)
- If data ready: lock Wire1 mutex, read measurement, unlock
- Per-sensor timestamps track data freshness
- Opponent detection: if distance < 1600mm, set bit in `opponentDirMask`

**Failure Handling:**
- If sensor fails init: mark `g_tofInitialized[i] = false`
- On poll: if sensor offline, set status 0xFF and invalid flag
- Core0 can detect offline sensors via `STATUS_TOF_OFFLINE` flag

**Tuning Parameters:**
- `TOF_TIMING_BUDGET_US`: 25000 (25ms) - trade accuracy vs. speed
- `TOF_STAGGER_OFFSET_MS`: 7 (ms) - prevents simultaneous data-ready events
- `DETECT_THRESH_MM`: 1600 (160cm) - opponent detection range

### Module B: Button Debouncing

**File Section:** Lines 683-754 in `BottleSumo_RP2040_DualCore.ino`

**Key Functions:**
- `sampleButtonsRaw()`: Sample pins every 5ms
- `debounceButtons()`: Process 20ms history

**Algorithm:**
1. **Sampling (5ms period):**
   - Read raw pin states (active-low with pullup)
   - Store in circular history buffer: `rawHistory[button][sample]`
   - Advance history index modulo 4

2. **Debouncing (20ms period):**
   - For each button, check if all 4 samples match
   - If yes and value = 1 (pressed): set bit in `stableMask`
   - Compute edge mask: `edgeMask = (newStable XOR oldStable) AND newStable`
   - Copy to snapshot, clear edgeMask

**Current Status:** **STUB IMPLEMENTATION**
- Raw sampling returns 0 (no hardware connected)
- Debouncing clears masks
- TODO markers indicate where to add pin reads

**Future Integration:**
- Uncomment pin configuration in `initCore1()`
- Uncomment raw reads in `sampleButtonsRaw()`
- Update `BUTTON_COUNT` if more than 2 buttons needed

### Module C: IR Acquisition + Edge Detection

**File Section:** Lines 756-830 in `BottleSumo_RP2040_DualCore.ino`

**Key Function:**
- `readIRSensorsAndDerive(snapshot)`: Read sensors, compute metrics

**Algorithm:**
1. **Read ADS1115 (4 channels):**
   - `readADC_SingleEnded(i)` for i ∈ [0,3]
   - Convert raw to volts: `volts = raw × 0.125mV × 0.001`
   - Store in `snapshot.irRaw[i]` and `snapshot.irVolts[i]`

2. **Snapshot Thresholds:**
   - Copy from `g_commandBlock.thresholds[]` (thread-safe with DMB)
   - Ensures consistent threshold comparison within single snapshot

3. **Edge Detection:**
   - For each sensor: `sensorOverThreshold[i] = (volts[i] > threshold[i])`
   - Count exceeding sensors → `dangerLevel` (0-4)
   - Set `edgeDetected = (dangerLevel > 0)`

4. **Direction Derivation (Priority Logic):**
   ```
   Priority 1: FRONT (both front) or BACK (both back)
   Priority 2: LEFT (both left) or RIGHT (both right)
   Priority 3: Diagonals (FRONT_LEFT, FRONT_RIGHT, etc.)
   Default: SAFE
   ```

**Timing:**
- ADS1115 at 860 SPS → ~1.16ms per 4-channel read
- Processing overhead: ~0.3ms
- Total WCET: ~1.5ms

**Calibration:**
- Threshold defaults: 2.5V (white line detection)
- Adjust per-sensor via TCP command: `{"cmd":"set_threshold","sensor":i,"value":v}`

### Module D: JSON Telemetry Builder

**File Section:** Lines 832-906 in `BottleSumo_RP2040_DualCore.ino`

**Key Function:**
- `buildTelemetryJSON(snapshot)`: Construct JSON string

**Output Format (Short Keys):**
```json
{
  "seq": 12345,           // Sequence number
  "tIr": 98765,           // IR timestamp (ms)
  "tTof": 98760,          // ToF timestamp (ms)
  "irV": [1.23, 2.34, 0.56, 1.78],  // IR voltages
  "irR": [1234, 2345, 567, 1789],   // IR raw ADC values
  "th": [2.5, 2.5, 2.5, 2.5],       // Thresholds snapshot
  "dLvl": 2,              // Danger level (0-4)
  "eDir": 1,              // Edge direction enum (0-8)
  "edg": true,            // Edge detected flag
  "tofD": [450, 1200, 800],  // ToF distances (mm)
  "tofV": 7,              // ToF valid mask (bits 0-2)
  "opp": 5,               // Opponent direction mask
  "btn": 0,               // Button stable mask
  "btnE": 0,              // Button edge mask
  "st": 0                 // Status flags
}
```

**Implementation Details:**
- Uses static buffer: `char g_telemetryBuffer[512]`
- No dynamic allocation (no String class)
- `snprintf()` for formatting (safe, bounded)
- Newline-delimited for TCP streaming
- WCET: ~5ms (string formatting overhead)

**Bandwidth:**
- JSON size: ~250 bytes per message
- Frequency: 20 Hz
- Total: 5 KB/s (very low, WiFi handles easily)

### Module E: Core1 Integrated Scheduler

**File Section:** Lines 908-1125 in `BottleSumo_RP2040_DualCore.ino`

**Key Functions:**
- `initCore1()`: Initialize hardware and task timing
- `loopCore1()`: Execute tasks based on due timestamps

**Scheduler Design:**
- **Non-Preemptive:** Tasks execute to completion (all are short)
- **Deadline-Monotonic:** Implicit priority via period (shorter period = higher priority)
- **Task Table:** Each task has `nextDue` timestamp
- **No Delay:** All timing via `if (millis() >= nextDue)`

**Task Execution Flow:**
```cpp
void loopCore1() {
  now = millis();
  
  // Always execute (every loop)
  pollCommands();
  
  // Periodic tasks (if due)
  if (now >= nextIR) {
    readIRSensorsAndDerive();
    nextIR += IR_PERIOD_MS;
  }
  
  if (now >= nextButtonSample) {
    sampleButtonsRaw();
    nextButtonSample += BUTTON_SAMPLE_MS;
  }
  
  // ... (all other tasks)
  
  g_core1LoopCount++;  // Performance metric
}
```

**Scheduling Guarantees:**
- **Jitter:** ≤1 loop iteration (typically <2ms)
- **Overrun Handling:** If task takes longer than period, next occurrence skipped
- **Task Independence:** Tasks don't block each other (except Wire1 mutex)

**Performance Monitoring:**
- `g_core1LoopCount`: Incremented each loop
- Statistics task (1 Hz): Prints loop frequency via `loopCount / millis()`
- Typical: ~500 Hz (2ms/loop)
- During heavy load: ~100 Hz (10ms/loop with all tasks due)

---

## Tuning Parameters

### Critical Timing Parameters

| Parameter | Default | Valid Range | Impact | Tuning Advice |
|-----------|---------|-------------|--------|---------------|
| `IR_PERIOD_MS` | 10 | 5-50 | Edge detection latency | Reduce for faster edge avoidance (min 5ms for ADS1115) |
| `TOF_PERIOD_MS` | 20 | 15-100 | Opponent tracking update rate | Must be ≥ timing budget (25ms) |
| `TOF_TIMING_BUDGET_US` | 25000 | 15000-50000 | ToF accuracy vs. speed | Increase for long-range accuracy, decrease for speed |
| `SNAPSHOT_PERIOD_MS` | 10 | 5-50 | Core0 data freshness | Match or exceed IR_PERIOD_MS |
| `TELEMETRY_PERIOD_MS` | 50 | 20-1000 | WiFi bandwidth vs. latency | Decrease for real-time monitoring, increase to save power |

### Detection Thresholds

| Parameter | Default | Valid Range | Impact | Tuning Advice |
|-----------|---------|-------------|--------|---------------|
| `IR_EDGE_THRESHOLD_DEFAULT` | 2.5 V | 0.1-4.0 | Edge sensitivity | Calibrate per ring surface (white tape brightness) |
| `DETECT_THRESH_MM` | 1600 | 300-2000 | Opponent detection range | Increase for early attack, decrease to reduce false positives |
| `TOF_MAX_VALID_RANGE_MM` | 1500 | 500-2000 | ToF measurement ceiling | Match to ring diameter (~154cm standard) |

### Staleness Tolerances

| Parameter | Default | Valid Range | Impact |
|-----------|---------|-------------|--------|
| `IR_STALE_MS` | 50 | 20-100 | IR data freshness flag |
| `TOF_STALE_MS` | 150 | 50-300 | ToF data freshness flag |
| `SNAPSHOT_STALE_MS` | 40 | 20-100 | Core0 safety fallback trigger |

**Staleness Tuning:**
- Set to 2-3× expected update period
- Triggers safety fallback (motor stop) on Core0
- Log warnings to diagnose sensor failures

### Button Debouncing

| Parameter | Default | Valid Range | Impact |
|-----------|---------|-------------|--------|
| `BUTTON_SAMPLE_MS` | 5 | 1-10 | Sample rate |
| `BUTTON_DEBOUNCE_MS` | 20 | 10-50 | Debounce window |
| `DEBOUNCE_SAMPLES_REQUIRED` | 4 | 2-8 | Stability threshold |

**Calculation:** `BUTTON_DEBOUNCE_MS = BUTTON_SAMPLE_MS × DEBOUNCE_SAMPLES_REQUIRED`

---

## Memory Footprint Analysis

### Static Memory Allocation

| Structure/Buffer | Size (Bytes) | Count | Total |
|------------------|--------------|-------|-------|
| `SensorSnapshot` | 128 | 4 | 512 |
| `SensorSnapshotExchange` | 256 + 8 | 1 | 264 |
| `CommandBlock` | 32 | 1 | 32 |
| `g_telemetryBuffer` | 512 | 1 | 512 |
| `ButtonDebounceState` | ~40 | 1 | 40 |
| `Core1TaskState` | ~180 | 1 | 180 |
| `Core0Context` | ~140 | 1 | 140 |
| Hardware instances (ADS, VL53L0X) | ~200 | 4 | 800 |
| Mutexes | 8 | 1 | 8 |
| **Total Static** | | | **~2.5 KB** |

### Stack Usage (Estimated)

| Core | Typical Stack Depth | Peak Stack (WCET) | Notes |
|------|---------------------|-------------------|-------|
| Core0 | ~1 KB | ~2 KB | State machine + WiFi (future) |
| Core1 | ~2 KB | ~4 KB | I2C transactions, snprintf |
| **Total** | **~3 KB** | **~6 KB** | Well below 264 KB limit |

### Flash Usage (Estimated)

| Component | Size (Bytes) |
|-----------|--------------|
| Compiled code (.text) | ~40 KB |
| Adafruit libraries | ~30 KB |
| Arduino-pico core | ~50 KB |
| WiFi stack (future) | ~100 KB |
| **Total** | **~220 KB** |

**Available Flash:** 2 MB (2048 KB) → **89% free**

---

## Compilation & Upload Instructions

### Prerequisites

1. **Install Arduino IDE** (v2.x recommended) or **Platform.IO**
2. **Install arduino-pico core:**
   - Open Arduino IDE → Preferences
   - Add board manager URL:  
     `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
   - Tools → Board → Boards Manager → Search "pico" → Install

3. **Install Required Libraries** (via Library Manager):
   - Adafruit ADS1X15
   - Adafruit VL53L0X
   - Adafruit GFX
   - Adafruit SSD1306

### Compilation Steps

1. Open `BottleSumo_RP2040_DualCore.ino` in Arduino IDE
2. Select Board:
   - Tools → Board → Raspberry Pi Pico/RP2040 → **Raspberry Pi Pico W**
3. Configure Board Options:
   - CPU Speed: **125 MHz** (default)
   - Flash Size: **2 MB (Sketch: 1 MB, FS: 1 MB)** (or no FS)
   - Debug Port: **Disabled** (or Serial if debugging)
   - Upload Method: **Default (UF2)**
4. Compile:
   - Sketch → Verify/Compile
   - Expected size: ~220 KB (~11% of flash)

### Upload Steps

**Method 1: UF2 Bootloader (Recommended)**
1. Hold BOOTSEL button on Pico W while plugging USB
2. Pico appears as USB drive "RPI-RP2"
3. Sketch → Upload
4. IDE copies .uf2 file, Pico auto-reboots

**Method 2: Picoprobe (Advanced)**
- Requires second Pico configured as debugger
- Enables single-click upload without BOOTSEL
- See arduino-pico documentation

### Verification

After upload, open Serial Monitor (115200 baud):
```
======================================
RP2040 Dual-Core Bottle Sumo Firmware
Version: 1.0.0
======================================

Core0: Initializing...
Core0: Ready
Waiting for Core1 initialization...

=== Core1 Initialization ===
ADS1115: OK
=== ToF Sensor Initialization (Continuous Mode) ===
RIGHT ToF: OK @ 0x30 (continuous mode)
FRONT ToF: OK @ 0x31 (continuous mode)
LEFT ToF: OK @ 0x32 (continuous mode)
All ToF sensors initialized successfully!
Core1: Ready

Core1: 487.3 Hz | Seq: 1023 | ToF: RFL
```

---

## Troubleshooting & Debugging

### Common Issues

#### 1. ADS1115 Not Found

**Symptoms:** `ERROR: ADS1115 init failed!` → Firmware halts

**Causes:**
- Incorrect I2C address (default 0x48, check ADDR pin)
- Loose wiring on SDA/SCL
- Wrong I2C bus (should use Wire, not Wire1)

**Solutions:**
1. Run I2C scanner on Wire bus (add debug code):
   ```cpp
   for (uint8_t addr = 1; addr < 127; addr++) {
     Wire.beginTransmission(addr);
     if (Wire.endTransmission() == 0) {
       Serial.printf("Found device at 0x%02X\n", addr);
     }
   }
   ```
2. Check solder joints on ADS1115 breakout
3. Verify 3.3V power supply

#### 2. ToF Sensors Offline

**Symptoms:** `WARNING: ToF sensor initialization incomplete` or `ToF: ---` in stats

**Causes:**
- XSHUT pin wiring incorrect (GP11/12/13)
- I2C address conflict (all default to 0x29 until reassigned)
- Wire1 mutex contention (OLED on same bus)

**Solutions:**
1. Check XSHUT wiring with multimeter:
   - All pins should be HIGH (~3.3V) after init
   - If LOW, check for short to ground
2. Add delay between sensor inits (increase `TOF_STAGGER_OFFSET_MS`)
3. Temporarily disable OLED init to isolate issue

#### 3. Stale Data Warnings

**Symptoms:** Serial prints `⚠️ 警告: 感測器數據過時!` (IR stale) or `⚠️ 警告: ToF 感測器數據過時!` (ToF stale)

**Causes:**
- Core1 loop too slow (check loop frequency in stats)
- I2C bus contention (Wire1 mutex held too long)
- Sensor read timeout

**Solutions:**
1. Reduce telemetry frequency: `TELEMETRY_PERIOD_MS = 100` (10 Hz)
2. Increase staleness tolerance: `IR_STALE_MS = 100`, `TOF_STALE_MS = 300`
3. Check Core1 loop frequency (should be >100 Hz):
   - If <50 Hz, tasks are taking too long
   - Reduce `TOF_TIMING_BUDGET_US` to 20000 (20ms)

#### 4. Compilation Errors

**Error:** `'mutex_t' does not name a type`

**Solution:** Ensure `#include <pico/mutex.h>` is present (it is in firmware)

**Error:** `'DMB' was not declared in this scope`

**Solution:** Check that `#define DMB()` macro is defined in Section 5

**Error:** Library not found (ADS1X15, VL53L0X, etc.)

**Solution:** Install missing libraries via Library Manager

### Debug Techniques

#### Enable Verbose Logging

Uncomment debug lines in key functions:

```cpp
// In readIRSensorsAndDerive():
Serial.printf("IR: %.2f %.2f %.2f %.2f | Edge: %d\n",
  snapshot.irVolts[0], snapshot.irVolts[1], snapshot.irVolts[2], snapshot.irVolts[3],
  snapshot.edgeDetected);

// In pollOneToFSensor():
Serial.printf("ToF[%d]: %u mm (status %u)\n", sensorIndex, measure.RangeMilliMeter, measure.RangeStatus);

// In loopCore1():
Serial.printf("Seq: %lu | IR: %lu | ToF: %lu\n", 
  g_core1State.localSequence, 
  g_core1State.draftSnapshot.irTimestamp, 
  g_core1State.draftSnapshot.tofTimestamp);
```

#### Monitor Telemetry JSON

Uncomment line in `loopCore1()`:

```cpp
if (now >= g_core1State.nextTelemetry) {
  buildTelemetryJSON(g_core1State.draftSnapshot);
  Serial.print(g_telemetryBuffer);  // <-- UNCOMMENT THIS
  g_core1State.nextTelemetry += Config::TELEMETRY_PERIOD_MS;
}
```

Output:
```json
{"seq":1234,"tIr":98765,"tTof":98760,"irV":[0.123,0.234,0.567,0.789], ...}
```

#### Check Memory Barriers

If seeing inconsistent data (rare), verify memory barriers:

```cpp
// Add after critical writes:
Serial.println("Before DMB");
DMB();
Serial.println("After DMB");
```

#### Core Synchronization Test

Add to `loop()` (Core0):

```cpp
static uint32_t lastSeq = 0;
if (g_core0Context.latestSnapshot.sequence != lastSeq) {
  Serial.printf("Core0 received seq %lu (age: %lu ms)\n",
    g_core0Context.latestSnapshot.sequence,
    millis() - g_core0Context.latestSnapshot.captureMillis);
  lastSeq = g_core0Context.latestSnapshot.sequence;
}
```

Expected output: New sequence every 10ms, age <2ms

---

## Future Enhancements

### Planned Features (Not Yet Implemented)

1. **WiFi Access Point & TCP Server:**
   - Enable WiFi AP mode with `WIFI_AP_SSID` and `WIFI_AP_PASSWORD`
   - TCP server on port 4242 for telemetry streaming
   - JSON command reception: `{"cmd":"set_threshold","sensor":i,"value":v}`
   - Integration point: `loop()` in Core0, call `handleTCPClients()`

2. **OLED Display Updates:**
   - Show robot state, sensor readings, WiFi status
   - Update at 2-5 Hz (throttled to avoid Wire1 mutex contention)
   - Requires Wire1 mutex: `mutex_enter_blocking(&g_wire1Mutex)`

3. **Motor Driver Integration:**
   - Apply `g_commandBlock.motorLeft` and `motorRight` to PWM outputs
   - Requires H-bridge driver (L298N, DRV8833, or similar)
   - Add to `pollCommands()` in Core1

4. **Button Functionality:**
   - Start/stop autonomous mode
   - Emergency stop
   - Threshold calibration mode
   - Uncomment stubs in Module B when hardware added

5. **Advanced State Machine:**
   - Implement full competition strategy (current is stub)
   - Add states: IDLE, SEARCH, ATTACK, RETREAT, RECOVER
   - Integrate opponent tracking with ToF sensors

### Performance Optimizations

- **DMA for ADS1115:** Reduce IR read time from 1.5ms to <0.5ms
- **ToF Interrupt Mode:** Poll only when INT pin asserts (reduce CPU by ~5%)
- **SIMD Edge Detection:** Use RP2040 interpolator for parallel threshold checks

---

## License & Credits

**License:** MIT (or project-specific license)

**Author:** CTEA-BottleSumo Project (Autonomous Copilot Agent)

**Libraries Used:**
- Adafruit ADS1X15 (MIT License)
- Adafruit VL53L0X (MIT License)
- arduino-pico core by Earle F. Philhower III (LGPL)

**Acknowledgments:**
- RP2040 dual-core architecture inspired by Raspberry Pi Pico SDK examples
- Lockless double-buffering pattern adapted from real-time systems literature

---

**END OF DOCUMENTATION**
