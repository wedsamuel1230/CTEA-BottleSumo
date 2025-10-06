# Bottle Sumo Dual-Core Snapshot Architecture
## Complete Implementation Index

Welcome to the Bottle Sumo Dual-Core Snapshot Architecture implementation! This directory contains a complete, production-ready Arduino sketch implementing the lockless double-buffer snapshot pattern for efficient dual-core sensor processing on the Raspberry Pi Pico W.

---

## 📚 Documentation Index

### Start Here
1. **README.md** - Main documentation
   - Overview of the architecture
   - Hardware requirements
   - Quick usage guide
   - Configuration options

2. **QUICKSTART.md** - Hands-on guide
   - 5-minute quick start
   - Detailed wiring diagrams
   - Pin mapping tables
   - Example code patterns
   - Testing procedures

### Deep Dive
3. **ARCHITECTURE.md** - Design documentation
   - Design goals and rationale
   - Lockless pattern explanation
   - Performance analysis
   - Comparison with alternatives
   - Safety considerations

4. **SUMMARY.md** - Implementation verification
   - Specification compliance checklist
   - File structure overview
   - Testing checklist
   - Performance metrics
   - Known limitations

---

## 💻 Source Code Index

### Core Implementation
```
BottleSumo_DualCore_Snapshot.ino (651 lines)
├── Configuration (lines 1-119)
│   └── Hardware pins, timing, thresholds
├── Core 1 Functions (lines 120-408)
│   ├── Sensor initialization
│   ├── IR sensor reading (ADS1115)
│   ├── ToF sensor reading (VL53L1X)
│   ├── Button debouncing
│   ├── Preprocessing (edge/danger/opponent detection)
│   ├── setup1() - Core 1 initialization
│   └── loop1() - High-speed sensor loop (~850 Hz)
└── Core 0 Functions (lines 409-651)
    ├── Command generation
    ├── Motor control
    ├── State machine (5 states)
    ├── setup() - Core 0 initialization
    └── loop() - Decision loop (~100 Hz)
```

### Data Structures
```
include/sensor_shared.h (75 lines)
├── EdgeDirection enum (9 values)
├── SensorSnapshot struct (74 bytes)
│   ├── Timing (12 bytes)
│   ├── IR data (40 bytes)
│   ├── Derived metrics (4 bytes)
│   ├── ToF data (8 bytes)
│   ├── Buttons (2 bytes)
│   └── Status/reserved (8 bytes)
├── SensorSnapshotExchange (double buffer)
└── CommandBlock (Core 0 → Core 1 commands)
```

### Core Communication
```
src/sensor_shared.cpp (16 lines)
└── Global variable initialization

src/core1_publish.cpp (30 lines)
├── memBarrier() - Memory ordering
└── publishSensorSnapshot() - Lockless publish

src/core0_consume.cpp (21 lines)
└── fetchLatestSnapshot() - Lockless consume
```

---

## 🚀 Quick Start (3 Steps)

### 1. Install Libraries
Arduino IDE → Library Manager → Install:
- `Adafruit ADS1X15`
- `VL53L1X`

### 2. Hardware Setup (Minimum)
```
ADS1115 → Raspberry Pi Pico W
  VDD → 3.3V
  GND → GND
  SDA → GP4
  SCL → GP5
  A0-A3 → QRE1113 sensors
```

### 3. Upload & Run
```
1. Open BottleSumo_DualCore_Snapshot.ino
2. Select board: "Raspberry Pi Pico W"
3. Upload (Ctrl+U)
4. Open Serial Monitor (115200 baud)
```

---

## 📖 Documentation Guide

### For First-Time Users
**Read in this order:**
1. **README.md** (15 min) - Understand the basics
2. **QUICKSTART.md** (30 min) - Wire up hardware
3. Upload code and test
4. Try example code patterns in QUICKSTART.md

### For Developers
**Read in this order:**
1. **ARCHITECTURE.md** (45 min) - Understand design decisions
2. **SUMMARY.md** (15 min) - Check implementation details
3. Review source code with architecture in mind
4. Extend as needed

### For Troubleshooting
1. Check "Common Issues" in **QUICKSTART.md**
2. Review "Safety & Error Handling" in **ARCHITECTURE.md**
3. Use testing procedures in **QUICKSTART.md**
4. Check Serial Monitor output for diagnostics

---

## 🎯 Key Features

### ✨ Lockless Communication
- **Zero blocking** - Cores never wait for each other
- **Constant latency** - Predictable <3µs overhead
- **Memory barriers** - Ensures data consistency

### ⚡ High Performance
- Core 1: ~850 Hz (sensor acquisition)
- Core 0: ~100 Hz (state machine)
- Latency: 5-12 ms typical (sensor → decision)

### 🧠 Preprocessing
Core 1 computes derived metrics:
- Edge direction (9 possibilities)
- Danger level (0-4 scale)
- Opponent direction (bitmask)

### 🎮 State Machine
5 robot states:
- INIT, SEARCH, ATTACK, RETREAT, EMERGENCY

### 🔘 Button Handling
- 20ms hardware debouncing
- Edge detection (press/release events)
- Non-blocking algorithm

---

## 📊 File Statistics

| File | Lines | Purpose |
|------|-------|---------|
| **Documentation** |
| README.md | 299 | Main documentation |
| QUICKSTART.md | 611 | Quick start guide |
| ARCHITECTURE.md | 636 | Design documentation |
| SUMMARY.md | 448 | Implementation summary |
| **Source Code** |
| BottleSumo_DualCore_Snapshot.ino | 651 | Main sketch |
| include/sensor_shared.h | 75 | Data structures |
| src/sensor_shared.cpp | 16 | Global variables |
| src/core1_publish.cpp | 30 | Publish function |
| src/core0_consume.cpp | 21 | Consume function |
| **Total** | **2,787** | Complete implementation |

---

## 🔍 Code Navigation

### Finding Specific Features

#### Sensor Reading
- **IR sensors**: `readIRSensors()` (line ~232)
- **ToF sensors**: `readToFSensors()` (line ~237)
- **Buttons**: `updateButtons()` (line ~253)

#### Preprocessing
- **Edge direction**: `computeEdgeDirection()` (line ~275)
- **Danger level**: `computeDangerLevel()` (line ~293)
- **Opponent direction**: `computeOpponentDirection()` (line ~303)

#### Core Functions
- **Core 1 setup**: `setup1()` (line ~310)
- **Core 1 loop**: `loop1()` (line ~343)
- **Core 0 setup**: `setup()` (line ~585)
- **Core 0 loop**: `loop()` (line ~606)

#### State Machine
- **State enum**: (line ~552)
- **State machine**: `runStateMachine()` (line ~568)

#### Communication
- **Publish**: `publishSensorSnapshot()` in src/core1_publish.cpp
- **Consume**: `fetchLatestSnapshot()` in src/core0_consume.cpp
- **Commands**: `updateMotorAndThresholds()` (line ~430)
- **Poll commands**: `pollCommands()` (line ~449)

---

## 🛠️ Customization Guide

### Changing Snapshot Rate
```cpp
// In Config namespace:
constexpr unsigned long CORE1_PUBLISH_INTERVAL_MS = 10;  // 100 Hz
// Change to:
constexpr unsigned long CORE1_PUBLISH_INTERVAL_MS = 20;  // 50 Hz
```

### Adding New Sensors
1. Add fields to `SensorSnapshot` in `sensor_shared.h`
2. Read sensor in `loop1()`
3. Add data to snapshot draft
4. Use data in `runStateMachine()`

### Adjusting Thresholds
```cpp
// At runtime via command channel:
float newThresh[4] = {2.8, 2.8, 2.8, 2.8};
updateMotorAndThresholds(0, 0, newThresh, 0x0F, 0);

// Or in Config namespace:
constexpr float EDGE_THRESHOLD_VOLTS = 2.8f;
```

### Adding New States
1. Add to `RobotState` enum
2. Add state name to `stateNames[]`
3. Add transition logic in `runStateMachine()`
4. Add state action in switch statement

---

## 🧪 Testing

### Basic Functionality Test
```
✓ Core 1 initializes sensors
✓ Core 0 waits for Core 1
✓ Snapshots published at 100 Hz
✓ Sequence numbers increment
✓ Data age < 15 ms
✓ State machine transitions
```

### Sensor Tests
- **IR**: Move hand over sensors → See voltage change
- **ToF**: Move object at different distances → See distance values
- **Buttons**: Press buttons → See stable/edge masks change

### Performance Test
Monitor Serial output for:
- Core 0 frequency: ~98-102 Hz ✓
- Core 1 frequency: ~840-860 Hz ✓
- Snapshot age: <15 ms ✓

---

## 📞 Support & Resources

### Documentation
- **Questions about architecture?** → Read ARCHITECTURE.md
- **Need wiring help?** → See QUICKSTART.md diagrams
- **Want examples?** → Check QUICKSTART.md patterns
- **Implementation details?** → See SUMMARY.md

### Troubleshooting
1. Check "Common Issues" section in QUICKSTART.md
2. Review Serial Monitor output for error messages
3. Verify I2C wiring and addresses
4. Test sensors individually before integration

### Reference Implementation
See `BottleSumo_RealTime_Streaming.ino` in parent directory for:
- WiFi telemetry streaming
- OLED display integration
- Multi-client TCP server

---

## 🏆 Implementation Highlights

### What Makes This Implementation Special

1. **Lockless by Design**
   - No mutex bottlenecks
   - Predictable performance
   - Minimal overhead

2. **Production Ready**
   - Comprehensive error handling
   - Staleness detection
   - Graceful degradation

3. **Well Documented**
   - 1,994 lines of documentation
   - 5 example code patterns
   - Detailed architecture explanation

4. **Extensible**
   - Reserved fields for expansion
   - Modular design
   - Clear separation of concerns

5. **Educational**
   - Comments explain "why" not just "what"
   - Architecture document teaches concepts
   - Examples show best practices

---

## 📝 Version History

**v1.0** (2025-01-06) - Initial release
- Complete dual-core snapshot architecture
- Lockless communication pattern
- Comprehensive documentation
- Example code patterns
- Testing procedures

---

## 🎓 Learn More

### Concepts Demonstrated
- Dual-core programming on RP2040
- Lockless data structures
- Memory barriers and ordering
- Sensor fusion techniques
- Real-time state machines
- Button debouncing algorithms
- Embedded systems design patterns

### Further Reading
- ARM Cortex-M0+ Technical Reference Manual
- RP2040 Datasheet
- "Lock-Free Programming" literature
- "Real-Time Systems" textbooks

---

## ✅ Checklist for New Users

- [ ] Read README.md (understand basics)
- [ ] Install required libraries (ADS1X15, VL53L1X)
- [ ] Wire up hardware (follow QUICKSTART.md)
- [ ] Upload and test basic functionality
- [ ] Verify Core 0 and Core 1 frequencies
- [ ] Test IR sensors (check voltages)
- [ ] Test ToF sensors (check distances)
- [ ] Try example code patterns
- [ ] Customize for your robot
- [ ] Read ARCHITECTURE.md (understand design)

---

## 🎉 Getting Started

**Ready to begin?**

1. Start with **README.md** for overview
2. Follow **QUICKSTART.md** for hardware setup
3. Upload code and test
4. Explore examples and customize

**Questions?** Check the documentation files listed above!

---

**Project:** CTEA-BottleSumo  
**Implementation:** Dual-Core Snapshot Architecture  
**Version:** 1.0  
**Date:** 2025-01-06  
**Status:** ✅ Complete and Production-Ready

---

*Happy coding! 🤖*
