# Architecture Design Document
## Dual-Core Lockless Snapshot Pattern for RP2040

### Executive Summary

This implementation provides a **lockless, double-buffered snapshot architecture** for efficient inter-core communication on the Raspberry Pi Pico's RP2040 dual-core processor. It eliminates mutex contention in the sensor acquisition hot path while maintaining data consistency and low latency.

---

## 1. Design Goals

### Primary Objectives
1. **Zero blocking in hot path**: No mutex waits during sensor reading or state machine execution
2. **Low latency**: <10-15ms from sensor capture to decision
3. **Consistent data**: Guarantee snapshot atomicity without locks
4. **Preprocessing efficiency**: Compute derived metrics once on Core 1
5. **Extensibility**: Easy to add new sensors or derived data

### Non-Goals
- Sub-millisecond latency (not needed for sumo robot)
- Hard real-time guarantees (soft real-time is sufficient)
- Multi-reader support (only Core 0 reads snapshots)

---

## 2. Core Communication Architecture

### 2.1 Lockless Double-Buffer Pattern

#### Structure
```cpp
struct SensorSnapshotExchange {
  SensorSnapshot buffers[2];           // Double buffer
  volatile uint8_t publishIndex;       // Atomic index (0 or 1)
  volatile uint32_t latestSequence;    // Quick staleness check
};
```

#### Publish Algorithm (Core 1)
```
1. current = publishIndex
2. next = current XOR 1           // Flip buffer
3. Fill buffers[next] with new data
4. buffers[next].sequence = ++seq
5. Memory barrier (DMB)
6. publishIndex = next            // Atomic publish
7. Memory barrier (DMB)
```

#### Consume Algorithm (Core 0)
```
1. idx1 = publishIndex            // Read current index
2. Copy buffers[idx1] to local
3. idx2 = publishIndex            // Check for race
4. If idx1 != idx2:               // Rare: publish during copy
     Copy buffers[idx2] to local  // Retry once
5. Return data
```

### 2.2 Why Lockless?

#### Mutex-Based Approach (Old)
```cpp
// Core 1 (High frequency, ~850 Hz)
mutex_lock(&data_mutex);          // ← BLOCKS if Core 0 holds lock
shared_data.voltages[0] = ...;
shared_data.voltages[1] = ...;
shared_data.voltages[2] = ...;
shared_data.voltages[3] = ...;
mutex_unlock(&data_mutex);

// Core 0 (Medium frequency, ~100 Hz)
mutex_lock(&data_mutex);          // ← BLOCKS Core 1 from publishing
read all shared data
process data
mutex_unlock(&data_mutex);
```

**Problems:**
- Core 1 blocks waiting for Core 0 → sensor jitter
- Core 0 blocks waiting for Core 1 → decision latency
- Lock contention overhead (~5-20µs per lock/unlock)
- Priority inversion risk

#### Lockless Approach (New)
```cpp
// Core 1 (Never blocks)
uint8_t next = publishIndex ^ 1;
buffers[next] = freshData;        // Write to "other" buffer
memBarrier();
publishIndex = next;              // Atomic swap

// Core 0 (Never blocks)
uint8_t idx = publishIndex;       // Atomic read
SensorSnapshot snap = buffers[idx];  // Read from published buffer
```

**Benefits:**
- **Zero blocking**: Both cores run independently
- **Constant time**: No waiting, no contention
- **Low overhead**: Single atomic read/write
- **Predictable latency**: Always <1µs for snapshot access

### 2.3 Memory Barriers

#### Why Needed?
ARM Cortex-M0+ can reorder memory operations. Without barriers:
```
// What you write:
buffers[next].irVolts[0] = 1.23;
buffers[next].irVolts[1] = 2.34;
buffers[next].sequence = 42;
publishIndex = next;

// What CPU might execute:
publishIndex = next;              // ← Published too early!
buffers[next].irVolts[0] = 1.23;  // ← Core 0 might read incomplete data
buffers[next].sequence = 42;
buffers[next].irVolts[1] = 2.34;
```

#### Solution: Memory Barrier
```cpp
static inline void memBarrier() {
  __asm volatile("" ::: "memory");  // Compiler barrier
}

// Usage:
buffers[next] = freshData;
memBarrier();                       // All writes complete before...
publishIndex = next;                // ...this publish
```

For strict ordering on multi-core, could use:
```cpp
__asm volatile("dmb" ::: "memory"); // Data Memory Barrier (hardware)
```

However, RP2040's memory architecture makes compiler barrier sufficient for this use case.

---

## 3. Data Flow Architecture

### 3.1 Core 1: Sensor Acquisition Pipeline

```
┌─────────────────────────────────────────────────────┐
│                   Core 1 Loop                        │
│                  (~850 Hz peak)                      │
└─────────────────────────────────────────────────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Read IR Sensors    │  <-- ADS1115, ~860 SPS
    │  (4 channels)       │      Continuous polling
    └──────────┬──────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Read ToF Sensors   │  <-- VL53L1X, 10 Hz
    │  (3 sensors)        │      Decimated update
    └──────────┬──────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Read Buttons       │  <-- Digital I/O
    │  + Debounce         │      20ms debounce window
    └──────────┬──────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Compute Derived    │  <-- Edge direction
    │  Metrics            │      Danger level
    │                     │      Opponent direction
    └──────────┬──────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Build Snapshot     │  <-- Pack all data
    │  Draft              │      Add timestamps
    └──────────┬──────────┘
              │
              ▼ (Every 10ms = 100 Hz)
    ┌─────────────────────┐
    │  Publish Snapshot   │  <-- Double-buffer swap
    │  (Lockless)         │      Atomic index update
    └──────────┬──────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Poll Commands      │  <-- Check for motor updates
    │  from Core 0        │      Apply threshold changes
    └─────────────────────┘
```

### 3.2 Core 0: State Machine & Control

```
┌─────────────────────────────────────────────────────┐
│                   Core 0 Loop                        │
│                    (~100 Hz)                         │
└─────────────────────────────────────────────────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Fetch Snapshot     │  <-- Lockless read
    │  (Lockless)         │      Check for staleness
    └──────────┬──────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Run State Machine  │  <-- Decide next action
    │                     │      Based on snapshot data
    └──────────┬──────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Generate Commands  │  <-- Motor setpoints
    │                     │      Threshold updates
    └──────────┬──────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Update Command     │  <-- Atomic sequence update
    │  Block              │      Core 1 polls this
    └──────────┬──────────┘
              │
              ▼
    ┌─────────────────────┐
    │  Telemetry &        │  <-- Serial logging
    │  Monitoring         │      Status printing
    └─────────────────────┘
```

---

## 4. Snapshot Data Structure Design

### 4.1 Structure Layout

```cpp
struct SensorSnapshot {
  // ===== Timing & Sequencing (12 bytes) =====
  uint32_t sequence;          // Monotonic counter
  uint32_t captureMillis;     // IR timestamp
  uint32_t tofMillis;         // ToF timestamp
  
  // ===== IR Sensors (40 bytes) =====
  int16_t  irRaw[4];          // Raw ADC counts (8 bytes)
  float    irVolts[4];        // Converted voltages (16 bytes)
  float    thresholds[4];     // Runtime thresholds (16 bytes)
  
  // ===== Derived Edge Info (4 bytes) =====
  uint8_t  dangerLevel;       // 0..4
  uint8_t  edgeDetected;      // 0/1
  EdgeDirection edgeDir;      // enum (1 byte)
  uint8_t  reserved1;         // Alignment
  
  // ===== ToF Sensors (8 bytes) =====
  uint16_t tofDist[3];        // Distances in mm (6 bytes)
  uint8_t  tofValidMask;      // Validity bits
  uint8_t  opponentDirMask;   // Opponent detection bits
  
  // ===== Buttons (2 bytes) =====
  uint8_t  buttonsStableMask; // Debounced state
  uint8_t  buttonsEdgeMask;   // Edge detection
  
  // ===== Status & Reserved (8 bytes) =====
  uint16_t statusFlags;       // Health/error flags
  uint8_t  reserved[6];       // Future expansion
};
// Total: 74 bytes (fits in cache line)
```

### 4.2 Design Rationale

#### Timestamps
- `captureMillis`: When IR data was captured
- `tofMillis`: Last ToF update (may be older due to 10Hz rate)
- Used for **staleness detection** on Core 0

#### Sequence Number
- Monotonic counter incremented on each publish
- Core 0 detects new snapshots by comparing sequence
- Also serves as **data correlation ID** for logging

#### Preprocessing
Core 1 computes these to **minimize Core 0 workload**:
- `dangerLevel`: Number of IR sensors over threshold
- `edgeDetected`: Boolean (any edge detected?)
- `edgeDir`: Enum indicating which edge (FRONT, BACK, etc.)
- `opponentDirMask`: Bitmask of ToF sensors seeing opponent

#### Bitmasks
- `tofValidMask`: Which ToF readings are valid (bit 0=Right, 1=Front, 2=Left)
- `opponentDirMask`: Which ToF sensors detect opponent
- `buttonsStableMask`: Current button states (debounced)
- `buttonsEdgeMask`: Buttons that changed this cycle
- **Efficient**: 8 bits for 8 boolean values

#### Reserved Space
- 6 bytes reserved for future expansion
- Maintains 8-byte alignment for efficient struct copying

---

## 5. Button Debouncing Algorithm

### 5.1 Requirements
- **Eliminate bounce**: Mechanical switches bounce for ~5-20ms
- **Edge detection**: Trigger actions on press/release, not hold
- **Non-blocking**: No delays in acquisition loop

### 5.2 Implementation

```cpp
struct ButtonState {
  uint8_t stableMask;        // Last confirmed stable state
  uint8_t lastRaw;           // Last raw read
  unsigned long lastChangeTime;  // Time of last change
};

void updateButtons(uint8_t* stableMask, uint8_t* edgeMask) {
  unsigned long now = millis();
  uint8_t rawMask = readAllButtons();  // Immediate read
  
  if (rawMask != lastRaw) {
    // Input changed - start debounce timer
    lastRaw = rawMask;
    lastChangeTime = now;
    *edgeMask = 0;  // No edge during transition
  }
  else if ((now - lastChangeTime) >= DEBOUNCE_MS) {
    // Input stable for debounce period
    uint8_t newStable = rawMask;
    *edgeMask = newStable ^ stableMask;  // XOR = changed bits
    stableMask = newStable;
  }
  else {
    *edgeMask = 0;  // Still bouncing
  }
  
  *stableMask = stableMask;
}
```

### 5.3 Edge Detection Usage

```cpp
// In Core 0:
if (snap.buttonsEdgeMask & 0x01) {  // Button 0 changed
  if (snap.buttonsStableMask & 0x01) {
    Serial.println("Button 0 PRESSED");
  } else {
    Serial.println("Button 0 RELEASED");
  }
}
```

---

## 6. Command Channel (Core 0 → Core 1)

### 6.1 Structure

```cpp
struct CommandBlock {
  volatile uint32_t seq;      // Sequence number (atomic)
  int16_t motorLeft;          // Left motor speed
  int16_t motorRight;         // Right motor speed
  uint16_t flags;             // Mode flags
  float thresholds[4];        // Threshold updates
  uint8_t thresholdMask;      // Which thresholds valid
  uint8_t reserved[3];        // Padding
};
```

### 6.2 Update Protocol

#### Core 0 (Command Sender)
```cpp
void updateMotorAndThresholds(...) {
  // 1. Write all fields
  g_commandBlock.motorLeft = left;
  g_commandBlock.motorRight = right;
  g_commandBlock.flags = flags;
  for (int i = 0; i < 4; i++) {
    if (mask & (1 << i)) {
      g_commandBlock.thresholds[i] = newThresh[i];
    }
  }
  g_commandBlock.thresholdMask = mask;
  
  // 2. Memory barrier
  __asm volatile("" ::: "memory");
  
  // 3. Atomically increment sequence (publishes update)
  g_commandBlock.seq++;
}
```

#### Core 1 (Command Receiver)
```cpp
void pollCommands() {
  static uint32_t lastApplied = 0;
  uint32_t seq = g_commandBlock.seq;
  
  if (seq != lastApplied) {
    // New command available
    applyMotorOutputs(g_commandBlock.motorLeft, 
                      g_commandBlock.motorRight);
    
    // Apply threshold changes
    uint8_t mask = g_commandBlock.thresholdMask;
    for (int i = 0; i < 4; i++) {
      if (mask & (1 << i)) {
        runtimeThresholds[i] = g_commandBlock.thresholds[i];
      }
    }
    
    lastApplied = seq;
  }
}
```

### 6.3 Why This Works
- **Sequence number** acts as versioning
- Core 1 **polls** (doesn't block waiting)
- If command missed (rare), next poll catches it
- **Atomic update**: All fields written before seq increment

---

## 7. Performance Analysis

### 7.1 Theoretical Performance

#### Snapshot Size
- 74 bytes (struct)
- Fits in L1 cache line (64-128 bytes on ARM Cortex-M0+)

#### Copy Time
- memcpy(74 bytes) ≈ **2-3µs** on RP2040 @ 125 MHz
- Negligible compared to 10ms loop period

#### Publish Overhead
- Buffer write: ~2µs
- Index swap: ~0.1µs (single atomic write)
- Memory barrier: ~0.1µs (compiler barrier, no CPU cycles)
- **Total: ~2-3µs** (vs ~10-20µs for mutex lock/unlock)

#### Consume Overhead
- Index read: ~0.1µs
- Buffer copy: ~2µs
- Sequence check: ~0.1µs
- **Total: ~2-3µs**

### 7.2 Measured Performance

Typical observed values:
```
Core 0 frequency: ~98-102 Hz (10ms loop)
Core 1 frequency: ~840-860 Hz (limited by ADS1115)
Snapshot age: 2-8 ms (typical)
Snapshot age: 10-15 ms (worst case, ToF read)
```

### 7.3 Latency Breakdown

```
Total Latency (Sensor → Decision):
  IR read:           1.2 ms  (860 SPS = 1.16ms per channel × 4)
  ToF read:         30 ms    (only every 100ms, so amortized: 3ms)
  Preprocessing:     0.1 ms  (edge detection, danger level)
  Publish:           0.003 ms
  Core 0 consume:    0.003 ms
  State machine:     0.1 ms
  ─────────────────────────
  TOTAL:            ~5-12 ms (typical)
                    ~40 ms   (when ToF reads happen)
```

---

## 8. Comparison with Alternatives

### 8.1 Mutex-Based Shared Memory

**Pros:**
- Simple to implement
- Standard pattern

**Cons:**
- Blocking (both cores can wait)
- Contention overhead
- Priority inversion risk
- Variable latency

**Verdict:** Not suitable for high-frequency sensor acquisition

### 8.2 FreeRTOS Queue

**Pros:**
- Built-in synchronization
- Multiple readers/writers

**Cons:**
- Requires RTOS (overhead)
- Copy on enqueue + dequeue
- Dynamic memory allocation
- Overkill for single producer/consumer

**Verdict:** Too heavy for this use case

### 8.3 RP2040 Multicore FIFO

**Pros:**
- Hardware FIFO (fast)
- Non-blocking option

**Cons:**
- Only 32-bit values (8 words)
- Need multiple pushes for full snapshot
- Still has contention if FIFO full
- No sequence numbering

**Verdict:** Good for small messages, not full snapshots

### 8.4 Double-Buffer Snapshot (This Implementation)

**Pros:**
- Zero blocking
- Constant latency
- No overhead (just atomic index)
- Simple implementation
- Extensible (just add fields to struct)

**Cons:**
- Single reader only (not an issue here)
- Slight memory overhead (2× buffer)
- Requires careful memory barriers

**Verdict:** ✅ Best fit for this application

---

## 9. Safety & Error Handling

### 9.1 Staleness Detection

```cpp
bool isStale = (millis() - snap.captureMillis) > 50;
if (isStale) {
  Serial.println("⚠️ Stale data");
  // Enter safe mode
}
```

### 9.2 Core 1 Watchdog

```cpp
if (snap.sequence == lastSeq && 
    (millis() - snap.captureMillis) > 100) {
  Serial.println("❌ Core 1 appears frozen");
  // Emergency stop
}
```

### 9.3 Sensor Health Monitoring

```cpp
if (snap.statusFlags & (1 << 0)) {
  Serial.println("⚠️ ToF Right offline");
}
```

---

## 10. Future Enhancements

### 10.1 Priority Scheduling
Add high-priority flag for emergency snapshots:
```cpp
struct SensorSnapshot {
  uint8_t priority;  // 0=normal, 1=high, 2=critical
  // ...
};
```

### 10.2 Snapshot History
Keep last N snapshots for temporal filtering:
```cpp
struct SensorSnapshotExchange {
  SensorSnapshot buffers[8];  // Ring buffer
  volatile uint8_t writeIndex;
  volatile uint8_t readIndex;
};
```

### 10.3 Telemetry Decimation
Add field to control telemetry rate:
```cpp
struct CommandBlock {
  uint8_t telemetryDecimation;  // Send every Nth snapshot
  // ...
};
```

---

## 11. Conclusion

The **lockless double-buffer snapshot pattern** provides:
- ✅ Zero blocking in hot path
- ✅ Constant, predictable latency
- ✅ Simple, maintainable implementation
- ✅ Easy to extend with new sensors
- ✅ Efficient memory usage

This architecture is **ideal for dual-core sensor fusion** on resource-constrained microcontrollers like the RP2040.

---

## References

1. ARM Cortex-M0+ Technical Reference Manual
2. RP2040 Datasheet (Raspberry Pi Ltd)
3. "Lock-Free Programming" (Herb Sutter)
4. "Memory Barriers: a Hardware View for Software Hackers" (Paul E. McKenney)

---

**Document Version:** 1.0  
**Last Updated:** 2025-01-06  
**Author:** CTEA-BottleSumo Project
