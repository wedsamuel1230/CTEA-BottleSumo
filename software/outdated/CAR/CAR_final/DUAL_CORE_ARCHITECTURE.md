# Dual-Core Architecture - BottleSumo v2

## Overview
This implementation separates concerns between the two RP2040 cores for optimal real-time performance:

- **Core 0**: Pure state machine (decision logic only)
- **Core 1**: All I/O operations (time-sliced scheduler)

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                         CORE 0                              │
│                   STATE MACHINE                             │
│                                                             │
│  ┌──────────────┐                                          │
│  │   Read       │ → Sensor Data (from shared memory)       │
│  │   Sensors    │                                          │
│  └──────┬───────┘                                          │
│         │                                                   │
│         ▼                                                   │
│  ┌──────────────┐                                          │
│  │   Decision   │ → Edge Detection (highest priority)      │
│  │   Logic      │ → Target Tracking                        │
│  │              │ → State Transitions                      │
│  └──────┬───────┘                                          │
│         │                                                   │
│         ▼                                                   │
│  ┌──────────────┐                                          │
│  │   Write      │ → Motor Commands (to shared memory)      │
│  │   Commands   │                                          │
│  └──────────────┘                                          │
│                                                             │
│  Update Rate: 100Hz (10ms cycle)                           │
└─────────────────────────────────────────────────────────────┘

                        ↕ Mutex-Protected
                     Shared Memory
                        ↕

┌─────────────────────────────────────────────────────────────┐
│                         CORE 1                              │
│              TIME-SLICED I/O SCHEDULER                      │
│                                                             │
│  Cycle Time: 500ms (10 slots × 50ms)                       │
│                                                             │
│  Slot 0 (0ms):    ┌──────────────────┐                    │
│                   │  IR Sensor Read  │                     │
│                   └──────────────────┘                     │
│                                                             │
│  Slot 1 (50ms):   ┌──────────────────┐                    │
│                   │  ToF Sensor 0    │                     │
│                   └──────────────────┘                     │
│                                                             │
│  Slot 2 (100ms):  ┌──────────────────┐                    │
│                   │  IR Sensor Read  │                     │
│                   └──────────────────┘                     │
│                                                             │
│  Slot 3 (150ms):  ┌──────────────────┐                    │
│                   │  ToF Sensor 1    │                     │
│                   └──────────────────┘                     │
│                                                             │
│  Slot 4 (200ms):  ┌──────────────────┐                    │
│                   │  IR Read +       │                     │
│                   │  Button Sample   │                     │
│                   └──────────────────┘                     │
│                                                             │
│  Slot 5 (250ms):  ┌──────────────────┐                    │
│                   │  ToF Sensor 2 +  │                     │
│                   │  Button Debounce │                     │
│                   └──────────────────┘                     │
│                                                             │
│  Slot 6 (300ms):  ┌──────────────────┐                    │
│                   │  IR Sensor Read  │                     │
│                   └──────────────────┘                     │
│                                                             │
│  Slot 7 (350ms):  ┌──────────────────┐                    │
│                   │  ToF Sensor 3 +  │                     │
│                   │  Motor Update    │                     │
│                   └──────────────────┘                     │
│                                                             │
│  Slot 8 (400ms):  ┌──────────────────┐                    │
│                   │  WiFi/TCP        │                     │
│                   │  Handling        │                     │
│                   └──────────────────┘                     │
│                                                             │
│  Slot 9 (450ms):  ┌──────────────────┐                    │
│                   │  ToF Sensor 4    │                     │
│                   └──────────────────┘                     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Shared Memory Structures

### SensorData (Core 1 → Core 0)
```cpp
struct SensorData {
    float irVoltages[4];          // IR sensor voltages
    bool irValid;                 // IR data validity
    uint16_t tofDistances[5];     // ToF distances (mm)
    bool tofValid[5];             // ToF validity flags
    ButtonMode buttonMode;        // Button state
    uint32_t timestamp;           // Data timestamp
};
```

### MotorCommand (Core 0 → Core 1)
```cpp
struct MotorCommand {
    float leftSpeed;              // -100 to +100
    float rightSpeed;             // -100 to +100
    bool emergencyStop;           // Emergency stop flag
    uint32_t timestamp;           // Command timestamp
};
```

### TelemetryData (Both Cores → WiFi)
```cpp
struct TelemetryData {
    SensorData sensors;           // Latest sensor data
    MotorCommand motors;          // Latest motor commands
    uint8_t stateMode;            // Current state
    uint32_t timestamp;           // Telemetry timestamp
};
```

## State Machine (Core 0)

### States
1. **IDLE** - Waiting for start command
2. **CALIBRATING** - Sensor calibration mode
3. **SEARCHING** - Spinning to find target
4. **TRACKING** - Aligning with target
5. **ATTACKING** - Full-speed forward attack
6. **EDGE_AVOIDING** - Emergency edge escape

### State Transitions
```
IDLE ──[button press]──> SEARCHING
   │
   └──[calibrate]──> CALIBRATING

SEARCHING ──[target detected]──> TRACKING
   │
   └──[edge detected]──> EDGE_AVOIDING

TRACKING ──[centered]──> ATTACKING
   │     ──[lost target long]──> SEARCHING
   │     ──[edge detected]──> EDGE_AVOIDING
   │
   └──[lost alignment]──> TRACKING

ATTACKING ──[lost center]──> TRACKING
   │      ──[edge detected]──> EDGE_AVOIDING
   │
   └──[lost target]──> TRACKING

EDGE_AVOIDING ──[escape complete]──> SEARCHING
```

### Priority System
1. **Edge Detection** (Highest) - Always checked first
2. **Target Tracking** - Only when safe
3. **Search Pattern** - Default behavior

## Time-Sliced Scheduler (Core 1)

### Design Principles
- **Deterministic Timing**: Each slot executes every 50ms
- **No Blocking**: All I/O operations fit within slot budget
- **Distributed Load**: Heavy operations spread across slots
- **Continuous Sampling**: IR sensors read 4× per cycle (200Hz)

### Slot Budget Analysis

| Slot | Task | Est. Time | Budget | Margin |
|------|------|-----------|--------|--------|
| 0 | IR Read (4 channels) | ~5ms | 50ms | 45ms |
| 1 | ToF Read (1 sensor) | ~35ms | 50ms | 15ms |
| 2 | IR Read | ~5ms | 50ms | 45ms |
| 3 | ToF Read | ~35ms | 50ms | 15ms |
| 4 | IR Read + Button Sample | ~10ms | 50ms | 40ms |
| 5 | ToF Read + Button Debounce | ~40ms | 50ms | 10ms |
| 6 | IR Read | ~5ms | 50ms | 45ms |
| 7 | ToF Read + Motor Update | ~36ms | 50ms | 14ms |
| 8 | WiFi/TCP | ~10ms | 50ms | 40ms |
| 9 | ToF Read | ~35ms | 50ms | 15ms |

### Sensor Update Rates
- **IR Sensors**: 200Hz (every 125ms average, 4× per cycle)
- **ToF Sensors**: 2Hz (each sensor every 500ms)
- **Buttons**: 4Hz (sample + debounce)
- **Motors**: 2Hz (command update)
- **WiFi**: 2Hz (telemetry broadcast)

## Performance Characteristics

### Latency Analysis

#### Edge Detection Response Time
1. Edge occurs
2. Next IR read (worst case): 125ms
3. Core 0 reads data: <1ms
4. Core 0 decides: <1ms
5. Core 0 writes command: <1ms
6. Core 1 updates motors (worst case): 250ms
**Total worst-case latency: ~377ms**

Typical case (IR just read): ~127ms

#### Target Tracking Response Time
1. Target appears
2. ToF sensor reads (worst case): 450ms
3. Core 0 reads data: <1ms
4. Core 0 decides: <1ms
5. Core 0 writes command: <1ms
6. Core 1 updates motors (worst case): 250ms
**Total worst-case latency: ~703ms**

### CPU Utilization
- **Core 0**: ~1% (100Hz @ <0.1ms per cycle)
- **Core 1**: ~40% average (20ms used per 50ms slot)

## Advantages of This Architecture

### Core 0 (Pure State Machine)
✅ **Deterministic** - No I/O delays
✅ **Fast** - <100μs decision cycles
✅ **Testable** - Pure logic, easy to unit test
✅ **Predictable** - Fixed execution time
✅ **Debuggable** - Clear state transitions

### Core 1 (Time-Sliced I/O)
✅ **No blocking** - Smooth operation
✅ **Balanced load** - Even CPU distribution
✅ **Predictable timing** - Fixed slot schedule
✅ **Scalable** - Easy to add/remove tasks
✅ **Isolated failures** - One sensor failure doesn't affect others

### System-Wide
✅ **Separation of concerns** - Logic vs I/O
✅ **Real-time capable** - Bounded latency
✅ **Maintainable** - Clear architecture
✅ **Extensible** - Easy to add features
✅ **Robust** - Mutex-protected shared memory

## Comparison with Previous Architecture

### CAR_final.ino (v1)
❌ Core 0: Mixed I/O and logic
❌ Core 1: Only motor control
❌ Blocking operations in main loop
❌ Variable timing (30-500ms loops)
❌ Edge detection blocks tracking

### CAR_final_v2.ino (v2)
✅ Core 0: Pure logic only
✅ Core 1: All I/O operations
✅ No blocking operations
✅ Fixed timing (10ms + 50ms slots)
✅ Parallel edge detection and tracking

## Testing Strategy

### Core 0 Testing
1. **Mock sensor data** - Inject test patterns
2. **Verify state transitions** - Check state machine logic
3. **Verify motor commands** - Check output correctness
4. **Timing tests** - Ensure <10ms cycles

### Core 1 Testing
1. **Slot timing** - Verify 50ms cadence
2. **Sensor reads** - Check data validity
3. **Motor updates** - Verify PWM output
4. **WiFi throughput** - Check telemetry rate

### Integration Testing
1. **Mutex correctness** - No data races
2. **End-to-end latency** - Measure response times
3. **Long-run stability** - 24+ hour tests
4. **Competition scenarios** - Real-world validation

## Configuration

### Tuning Parameters
```cpp
// Core 0 timing
constexpr uint32_t STATE_MACHINE_INTERVAL_MS = 10;  // 100Hz

// Core 1 timing
constexpr uint32_t TIME_SLICE_MS = 50;              // Slot duration
constexpr uint8_t TOTAL_SLOTS = 10;                 // Total slots

// Detection thresholds
constexpr float IR_THRESHOLD_FRONT = 1.5f;
constexpr float IR_THRESHOLD_BACK = 3.0f;
constexpr uint16_t DETECT_MIN_MM = 70;
constexpr uint16_t DETECT_MAX_MM = 1000;

// Behavior parameters
constexpr float ESCAPE_SPEED = 50.0f;
constexpr float SEARCH_SPIN_SPEED = 35.0f;
constexpr float ATTACK_SPEED = 100.0f;
```

## Troubleshooting

### If Core 0 stops responding
- Check mutex deadlocks
- Verify state machine isn't stuck
- Check for infinite loops in decision logic

### If Core 1 misses slots
- Reduce sensor timing budgets
- Check I2C bus contention
- Verify WiFi isn't blocking

### If motors don't respond
- Check motor command timestamps
- Verify Core 1 slot 7 is executing
- Check shared memory integrity

### If sensors report stale data
- Check Core 1 is running
- Verify mutex acquisition
- Check sensor hardware

## Future Enhancements

1. **Adaptive slot timing** - Adjust based on actual task duration
2. **Priority preemption** - Allow critical tasks to interrupt
3. **Multi-level state machine** - Hierarchical states
4. **Predictive tracking** - Kalman filtering for target prediction
5. **Dynamic threshold tuning** - Auto-calibration during runtime
6. **Advanced telemetry** - Performance metrics and diagnostics

## Conclusion

This dual-core architecture provides:
- **Separation of concerns** between logic and I/O
- **Predictable, deterministic timing** for competition reliability
- **Maintainable, testable code** for iterative development
- **Room for expansion** with unused CPU cycles

The time-sliced scheduler ensures smooth, jitter-free operation while the pure state machine provides fast, reliable decision-making. This architecture is specifically designed for the demanding real-time requirements of robotic sumo competition.
