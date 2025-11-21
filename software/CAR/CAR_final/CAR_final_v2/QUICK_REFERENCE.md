# Quick Reference - Dual-Core Architecture

## File Structure
```
CAR_final/
├── CAR_final.ino              # Original implementation (v1)
├── CAR_final_v2.ino           # New dual-core architecture (v2)
├── Car.cpp / Car.h            # Motor control abstraction
├── Motor.cpp / Motor.h        # Low-level motor driver
├── ToFArray.cpp / ToFArray.h  # VL53L0X sensor array
├── Ads1115Sampler.cpp / .h    # IR sensor ADC interface
├── ButtonManager.h            # Button debouncing
├── DUAL_CORE_ARCHITECTURE.md  # Detailed architecture docs
└── MIGRATION_GUIDE.md         # v1 to v2 migration guide
```

## Key Concepts

### Core 0: State Machine (Logic Only)
```cpp
loop() {
    // 1. Read sensors from shared memory
    SensorData sensors;
    readSensorData(sensors);
    
    // 2. Make decisions (pure logic)
    if (detectEdge(sensors)) {
        transitionState(EDGE_AVOIDING);
    } else if (detectTarget(sensors)) {
        transitionState(TRACKING);
    }
    
    // 3. Write commands to shared memory
    writeMotorCommand(leftSpeed, rightSpeed);
    
    delay(10);  // 100Hz update rate
}
```

### Core 1: Time-Sliced I/O
```cpp
loop1() {
    if (millis() - lastSlotTime >= 50) {  // Every 50ms
        executeSlot(currentSlot);         // Run slot task
        currentSlot = (currentSlot + 1) % 10;  // Next slot
        lastSlotTime = millis();
    }
}
```

## State Machine States

| State | Description | Entry Condition | Exit Condition |
|-------|-------------|-----------------|----------------|
| IDLE | Waiting for start | Button press or reset | RUN button pressed |
| CALIBRATING | Sensor calibration | TEST button pressed | Calibration complete |
| SEARCHING | Spinning to find target | No target visible | Target detected |
| TRACKING | Aligning with target | Target detected | Centered or lost |
| ATTACKING | Full-speed forward | Target centered | Lost alignment |
| EDGE_AVOIDING | Emergency escape | Edge detected | Escape complete |

## I/O Scheduler Slots

| Slot | Time | Task | Duration |
|------|------|------|----------|
| 0 | 0ms | IR Sensor Read | ~5ms |
| 1 | 50ms | ToF Sensor 0 | ~35ms |
| 2 | 100ms | IR Sensor Read | ~5ms |
| 3 | 150ms | ToF Sensor 1 | ~35ms |
| 4 | 200ms | IR Read + Button Sample | ~10ms |
| 5 | 250ms | ToF 2 + Button Debounce | ~40ms |
| 6 | 300ms | IR Sensor Read | ~5ms |
| 7 | 350ms | ToF 3 + Motor Update | ~36ms |
| 8 | 400ms | WiFi/TCP Handling | ~10ms |
| 9 | 450ms | ToF Sensor 4 | ~35ms |

## Shared Memory API

### Reading Sensor Data (Core 0)
```cpp
SensorData sensors;
readSensorData(sensors);

// Access sensor data
float frontLeftIR = sensors.irVoltages[1];
uint16_t centerToF = sensors.tofDistances[2];
bool tofValid = sensors.tofValid[2];
ButtonMode mode = sensors.buttonMode;
```

### Writing Motor Commands (Core 0)
```cpp
// Normal command
writeMotorCommand(50.0f, -50.0f);  // Left 50%, Right -50%

// Emergency stop
writeMotorCommand(0.0f, 0.0f, true);
```

### Writing Sensor Data (Core 1)
```cpp
SensorData data;
readSensorData(data);  // Get current

// Update specific fields
data.irVoltages[0] = 1.23f;
data.timestamp = millis();

writeSensorData(data);
```

### Reading Motor Commands (Core 1)
```cpp
MotorCommand cmd;
readMotorCommand(cmd);

// Execute command
if (cmd.emergencyStop) {
    gCar.stop();
} else {
    gCar.setMotors(cmd.leftSpeed, cmd.rightSpeed);
}
```

## Pin Assignments

### Motors
```cpp
LEFT_MOTOR_PWM = 11    // PWM output
LEFT_MOTOR_DIR = 12    // Direction pin
RIGHT_MOTOR_PWM = 14   // PWM output
RIGHT_MOTOR_DIR = 15   // Direction pin
```

### ToF Sensors (VL53L0X)
```cpp
TOF_XSHUT[0] = 8  // Right 45°
TOF_XSHUT[1] = 7  // Right 23°
TOF_XSHUT[2] = 6  // Center 0°
TOF_XSHUT[3] = 5  // Left 23°
TOF_XSHUT[4] = 4  // Left 45°

I2C_SDA = 2
I2C_SCL = 3
```

### IR Sensors (ADS1115)
```cpp
ADS1115_ADDRESS = 0x48  // On Wire1
A0 = Back-Left
A1 = Front-Left
A2 = Front-Right
A3 = Back-Right
```

### Buttons (Optional)
```cpp
BUTTON_TEST = 16  // Test mode
BUTTON_RUN = 17   // Run mode
```

## Tuning Parameters

### Detection Thresholds
```cpp
IR_THRESHOLD_FRONT = 1.5f   // Volts (edge detection)
IR_THRESHOLD_BACK = 3.0f    // Volts (edge detection)
DETECT_MIN_MM = 70          // Minimum ToF distance
DETECT_MAX_MM = 1000        // Maximum ToF distance
BIAS_DEADZONE = 0.1f        // Target centering tolerance
```

### Motor Speeds
```cpp
ESCAPE_SPEED = 50.0f           // Edge escape speed
BACK_ESCAPE_SPEED = 100.0f     // Back edge escape (fast)
SEARCH_SPIN_SPEED = 35.0f      // Searching rotation speed
ALIGN_SPIN_SPEED = 32.0f       // Coarse alignment speed
ALIGN_FINE_SPIN_SPEED = 18.0f  // Fine alignment speed
ATTACK_SPEED = 100.0f          // Full attack speed
```

### Timing
```cpp
TIME_SLICE_MS = 50        // I/O slot duration
LOST_HOLD_MS = 2000       // Hold time before searching
STATE_UPDATE_HZ = 100     // Core 0 update rate
```

## WiFi Telemetry

### Connection
```
SSID: BottleSumo_AP
Password: sumobot123456
IP: 192.168.4.1
Port: 8080
```

### JSON Format
```json
{
  "t": 12345,                          // Timestamp (ms)
  "ir": [0.12, 1.45, 1.50, 0.10],      // IR voltages [A0, A1, A2, A3]
  "tof": [120, 115, 110, 118, 125],    // ToF distances (mm) [R45, R23, C, L23, L45]
  "m": [35.0, 35.0],                   // Motor speeds [left, right]
  "s": 3                               // State (0=IDLE, 3=SEARCHING, 4=TRACKING, etc.)
}
```

### Connect via netcat
```bash
nc 192.168.4.1 8080
```

### Python client example
```python
import socket
import json

sock = socket.socket()
sock.connect(('192.168.4.1', 8080))

while True:
    data = sock.recv(1024).decode()
    for line in data.split('\n'):
        if line:
            telemetry = json.loads(line)
            print(f"State: {telemetry['s']}, ToF Center: {telemetry['tof'][2]}mm")
```

## Debugging

### Serial Monitor Commands
```
// Check initialization
Expected: "ADS1115 OK", "ToF sensors online: 5/5", "Motors OK"

// Watch state transitions
Format: "[STATE] X -> Y"
Example: "[STATE] 3 -> 4" (SEARCHING -> TRACKING)
```

### Common Debug Patterns

#### Check sensor freshness
```cpp
void stateMachineUpdate() {
    SensorData sensors;
    readSensorData(sensors);
    
    if (millis() - sensors.timestamp > 1000) {
        Serial.println("WARNING: Stale sensor data!");
    }
}
```

#### Monitor slot timing
```cpp
void executeSlot(uint8_t slot) {
    uint32_t start = millis();
    
    switch(slot) {
        case 0: slot0_IR_Read(); break;
        // ... etc
    }
    
    uint32_t duration = millis() - start;
    if (duration > 40) {  // Warning if >80% of 50ms
        Serial.printf("Slot %d slow: %dms\n", slot, duration);
    }
}
```

#### Trace motor commands
```cpp
void writeMotorCommand(float left, float right, bool estop) {
    Serial.printf("[MOTOR] L=%.1f R=%.1f E=%d\n", left, right, estop);
    
    mutex_enter_blocking(&gMotorMutex);
    gSharedMotorCommand.leftSpeed = left;
    gSharedMotorCommand.rightSpeed = right;
    gSharedMotorCommand.emergencyStop = estop;
    gSharedMotorCommand.timestamp = millis();
    mutex_exit(&gMotorMutex);
}
```

## Performance Targets

### Latency Goals
- Edge detection response: <400ms
- Target tracking response: <700ms
- Motor command latency: <260ms
- State transition: <10ms

### CPU Utilization
- Core 0: <5% (leaving headroom for future features)
- Core 1: <60% (balanced load across slots)

### Reliability
- 0 crashes in 24-hour test
- 0 mutex deadlocks
- 100% sensor data validity

## Quick Troubleshooting

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Motors don't respond | Core 1 not running | Check setup1() output |
| Sensors always invalid | I2C not initialized | Check Wire1.begin() |
| WiFi won't connect | AP not started | Check WiFi.softAP() return |
| Robot stuck in one state | State machine logic bug | Add debug prints |
| Jerky movement | Slot timing overrun | Reduce task complexity |
| Frequent crashes | Mutex deadlock | Review mutex usage |

## Testing Checklist

- [ ] Sensors initialize (Serial Monitor)
- [ ] WiFi AP visible on phone/laptop
- [ ] Telemetry JSON streams via netcat
- [ ] Wheels spin in SEARCHING state
- [ ] Target detection changes state to TRACKING
- [ ] Edge detection triggers EDGE_AVOIDING
- [ ] Motors stop when placed on blocks
- [ ] 1-hour continuous operation test
- [ ] Competition scenario validation

## Resources

- **Architecture Details**: `DUAL_CORE_ARCHITECTURE.md`
- **Migration Guide**: `MIGRATION_GUIDE.md`
- **Original Code**: `CAR_final.ino`
- **New Code**: `CAR_final_v2.ino`

## Version History

| Version | Date | Changes |
|---------|------|---------|
| v1 | Original | Blocking architecture, mixed I/O and logic |
| v2 | Current | Dual-core time-sliced, pure state machine |

---

**Quick Start**: Upload `CAR_final_v2.ino`, open Serial Monitor, wait for "Ready" messages, press RUN button, observe state transitions.
