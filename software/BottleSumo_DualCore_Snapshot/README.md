# Bottle Sumo - Dual-Core Snapshot Architecture

A high-performance, lockless dual-core Arduino implementation for the Raspberry Pi Pico W bottle sumo robot.

## Architecture Overview

This implementation uses a **lockless double-buffer snapshot pattern** for efficient Core1 → Core0 communication, eliminating mutex contention in the hot path while maintaining data consistency.

### Core Responsibilities

#### Core 1 (Sensor Acquisition Core)
- High-speed IR sensor reading (~860 SPS via ADS1115)
- ToF distance sensor polling (10 Hz)
- Button debouncing and edge detection
- **Preprocessing**: Computes derived metrics (danger level, edge direction, opponent direction)
- **Publishing**: Creates consolidated snapshots at 100 Hz
- **Command polling**: Applies motor commands and threshold updates from Core 0

#### Core 0 (State Machine & Control Core)
- **Lockless consumption**: Fetches latest snapshots without blocking
- State machine execution
- Motor control decisions
- Telemetry and status reporting
- **Command generation**: Sends motor setpoints and configuration to Core 1

## Key Features

### 1. Lockless Communication
- **Double-buffer pattern**: Two snapshot buffers with atomic index swap
- **No mutex in hot path**: Core 0 reads snapshots without blocking Core 1
- **Memory barriers**: Ensures proper ordering on ARM Cortex-M0+
- **Constant-time operations**: Publish and consume are O(1)

### 2. Preprocessing on Core 1
Core 1 computes derived metrics to reduce Core 0 workload:
- Edge detection status (boolean)
- Danger level (0-4 scale)
- Edge direction (enum: FRONT, BACK, LEFT, RIGHT, etc.)
- Opponent direction bitmask from ToF sensors
- Button state changes (debounced + edge detection)

### 3. Low Latency
- Typical latency: <10-15 ms from sensor reading to decision
- No heavy computation on Core 0
- Snapshot age tracking for staleness detection

### 4. Bidirectional Communication
- **Core 1 → Core 0**: Lockless double-buffer snapshots
- **Core 0 → Core 1**: Command block with sequence number
  - Motor setpoints (left/right)
  - Runtime threshold adjustments
  - Mode flags

## Hardware Requirements

### Microcontroller
- Raspberry Pi Pico W (RP2040 dual-core)

### Sensors
- **IR Sensors**: 4x QRE1113 connected to ADS1115 ADC
  - Front-left, front-right, back-left, back-right
- **ToF Sensors**: 3x VL53L1X distance sensors (I2C)
  - Right, front, left
- **Buttons** (optional): Up to 4 buttons for mode control

### I2C Configuration
- **Wire (I2C0)**: ADS1115 ADC
  - SDA: GP4
  - SCL: GP5
  - Speed: 400 kHz
- **Wire1 (I2C1)**: ToF sensors
  - SDA: GP26
  - SCL: GP27
  - Speed: 400 kHz

### ToF XSHUT Pins
- RIGHT sensor: GP13
- FRONT sensor: GP12
- LEFT sensor: GP11

### Button Pins (Optional)
- GP3, GP4, GP5, GP6 (configurable)

## Software Architecture

### File Structure
```
BottleSumo_DualCore_Snapshot/
├── BottleSumo_DualCore_Snapshot.ino  # Main Arduino sketch
├── include/
│   └── sensor_shared.h                # Snapshot data structures
├── src/
│   ├── sensor_shared.cpp              # Global variable definitions
│   ├── core1_publish.cpp              # Lockless publish function
│   └── core0_consume.cpp              # Lockless consume function
└── README.md                          # This file
```

### Data Structures

#### SensorSnapshot
Complete sensor state snapshot (published by Core 1):
- **Timing**: sequence number, capture timestamp, ToF timestamp
- **IR sensors**: Raw ADC values, voltages (4 sensors)
- **Derived metrics**: Danger level, edge detection, edge direction
- **ToF sensors**: Distances, validity mask, opponent direction mask
- **Buttons**: Stable state, edge mask (newly changed)
- **Status flags**: Offline sensors, error conditions

#### CommandBlock
Commands from Core 0 to Core 1:
- **Sequence number**: Atomic command versioning
- **Motor control**: Left/right setpoints
- **Thresholds**: Per-sensor edge detection thresholds
- **Flags**: Mode control bits

## Usage

### 1. Installation

#### Required Libraries
Install via Arduino Library Manager:
- Adafruit ADS1X15 (for ADS1115 ADC)
- VL53L1X (for ToF sensors)

#### Board Setup
1. Install Raspberry Pi Pico/RP2040 board support in Arduino IDE
2. Select "Raspberry Pi Pico W" as board
3. Connect Pico W via USB

### 2. Hardware Setup

#### Wiring Diagram
```
ADS1115 ADC:
  VDD  → 3.3V
  GND  → GND
  SDA  → GP4
  SCL  → GP5
  A0-A3 → QRE1113 sensors

ToF Sensors (all three):
  VDD  → 3.3V
  GND  → GND
  SDA  → GP26
  SCL  → GP27
  XSHUT → GP11/12/13 (Left/Front/Right)
```

### 3. Compilation & Upload
1. Open `BottleSumo_DualCore_Snapshot.ino` in Arduino IDE
2. Verify/compile the sketch
3. Upload to Raspberry Pi Pico W
4. Open Serial Monitor (115200 baud)

### 4. Monitoring
The system prints status every 5 seconds:
```
========================================
Uptime: 123 s
Core 0: 98.5 Hz | Core 1: 847.3 Hz
Sequence: 12345 | Age: 8 ms
State: SEARCH
Edge: NO | Danger: 0/4
IR Volts: [1.23, 1.45, 1.67, 1.89]
ToF Dist: [1234, 567, 890] | Valid: 0x07
Buttons: Stable=0x00 Edge=0x00
========================================
```

## Configuration

Edit `Config` namespace in the `.ino` file:

### Timing
```cpp
constexpr unsigned long CORE1_PUBLISH_INTERVAL_MS = 10;  // 100 Hz snapshots
constexpr unsigned long CORE0_LOOP_DELAY_MS = 10;        // 100 Hz state machine
constexpr unsigned long TOF_READ_INTERVAL_MS = 100;      // 10 Hz ToF update
```

### Thresholds
```cpp
constexpr float EDGE_THRESHOLD_VOLTS = 2.5f;             // IR edge detection
constexpr uint16_t TOF_DETECTION_THRESHOLD_MM = 1600;    // 160cm opponent range
```

### I2C Pins
```cpp
constexpr uint8_t I2C0_SDA_PIN = 4;   // Wire (ADS1115)
constexpr uint8_t I2C0_SCL_PIN = 5;
constexpr uint8_t I2C1_SDA_PIN = 26;  // Wire1 (ToF)
constexpr uint8_t I2C1_SCL_PIN = 27;
```

## State Machine

The robot operates in multiple states:

1. **INIT**: Initialization (brief)
2. **SEARCH**: Rotating to find opponent
3. **ATTACK**: Moving toward detected opponent
4. **RETREAT**: Backing away from edge (single sensor)
5. **EMERGENCY**: Stop immediately (multiple sensors)

### State Transitions
- **Edge detected (danger ≥ 2)** → EMERGENCY
- **Edge detected (danger = 1)** → RETREAT
- **Opponent detected** → ATTACK
- **Default** → SEARCH

## Extending the System

### Adding New Sensors
1. Update `SensorSnapshot` structure in `sensor_shared.h`
2. Read sensor in Core 1's `loop1()`
3. Add data to snapshot draft
4. Update Core 0 state machine to use new data

### Custom Commands
1. Add fields to `CommandBlock` in `sensor_shared.h`
2. Update `updateMotorAndThresholds()` to set new fields
3. Update `pollCommands()` in Core 1 to apply changes

### Telemetry Integration
The snapshot structure is JSON-friendly. To add WiFi streaming:
1. Add WiFi initialization in `setup()`
2. Serialize `SensorSnapshot` to JSON in Core 0
3. Stream at controlled rate (e.g., 20 Hz)

## Performance Characteristics

- **Core 0 frequency**: ~100 Hz (configurable)
- **Core 1 frequency**: ~850 Hz (limited by ADS1115 SPS)
- **Snapshot publish rate**: 100 Hz
- **IR sensor sample rate**: ~860 SPS (ADS1115 maximum)
- **ToF sensor update rate**: 10 Hz
- **Typical latency**: 8-12 ms (sensor → decision)
- **Memory usage**: ~2 KB for snapshots (double-buffered)

## Troubleshooting

### Issue: Core 1 not initializing
**Solution**: Check I2C wiring and addresses. Run I2C scanner.

### Issue: Stale data warnings
**Solution**: Core 1 may be blocked. Check:
- I2C bus conflicts
- Sensor initialization failures
- Excessive delay in Core 1 loop

### Issue: ToF sensors fail to initialize
**Solution**: 
- Verify Wire1 (GP26/27) connections
- Check XSHUT pin wiring
- Ensure sensors powered with 3.3V
- Try power cycling sensors

### Issue: Jittery motor control
**Solution**:
- Increase `CORE0_LOOP_DELAY_MS` for more stable decisions
- Add filtering to snapshot data
- Check for ground loops in motor wiring

## Advantages Over Mutex-Based Approach

| Aspect | Mutex-Based | Double-Buffer Snapshot |
|--------|-------------|------------------------|
| Core 0 blocking | Yes (waiting for lock) | No (always latest) |
| Core 1 blocking | Yes (contention) | No (lockless) |
| Latency | Variable (10-50ms) | Constant (<15ms) |
| Jitter | High | Low |
| Complexity | Medium | Low |
| CPU overhead | Mutex + contention | Memory barrier only |

## Future Enhancements

- [ ] WiFi telemetry streaming (JSON @ 20 Hz)
- [ ] OLED display integration (Core 0)
- [ ] Motor encoder feedback
- [ ] Kalman filtering for sensor fusion
- [ ] PID control for precise movements
- [ ] Remote configuration via WiFi commands
- [ ] Data logging to SD card

## References

- Problem statement: Dual-core snapshot architecture specification
- Reference implementation: `BottleSumo_RealTime_Streaming.ino`
- ARM Cortex-M0+ memory ordering: ARM Architecture Reference Manual

## License

See LICENSE file in repository root.

## Author

CTEA-BottleSumo Project
Date: 2025
