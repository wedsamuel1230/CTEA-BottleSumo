# Motor and Test Mode Integration Summary

## Date: October 16, 2025

This document summarizes the integration of motor control and test mode functionality into the BottleSumo_TimeSliced firmware and viewer.py GUI.

---

## 🚀 Key Changes

### 1. **Arduino Firmware (BottleSumo_TimeSliced.ino)**

#### Added Includes:
- `Motor.h` - Motor PWM control class
- `TestModeCommon.h` - Shared test mode definitions
- `MotorTestMode.h` - Motor testing module
- `SensorTestMode.h` - Sensor testing module

#### Hardware Configuration:
```cpp
// Motor pins
MOTOR_LEFT_PWM_PIN  = GP6
MOTOR_LEFT_DIR_PIN  = GP7
MOTOR_RIGHT_PWM_PIN = GP8
MOTOR_RIGHT_DIR_PIN = GP9
MOTOR_PWM_FREQ      = 20kHz
```

#### ToF Timing Adjustment:
- **Changed from:** 30ms per sensor (150ms total for 5 sensors)
- **Changed to:** 50ms per sensor (250ms total for 5 sensors)
- **Reason:** Improved long-range stability and accuracy
- **Budget updated:** `BUDGET_TOF_READ = 250ms`

#### Time Budgets (Core 1 Loop):
```
TASK 1: ADS Read       - 10ms
TASK 2: ToF Read       - 250ms (UPDATED from 150ms)
TASK 3: ADS Read       - 10ms
TASK 4: Button Sample  - 5ms
TASK 5: Button Debounce- 20ms
TASK 6: ADS Read       - 10ms
TASK 7: Motor PWM      - 5ms  (NEW)
TASK 8: WiFi/TCP       - 50ms
TASK 9: ADS Read       - 10ms
---------------------------------
Total cycle time: ~370ms (~2.7Hz)
Previous: ~265ms (~3.77Hz)
```

#### Motor Command Structure:
```cpp
struct MotorCommand {
  int8_t left_speed;    // -100 to +100 (duty cycle %)
  int8_t right_speed;   // -100 to +100 (duty cycle %)
  bool emergency_stop;
  unsigned long timestamp;
};
```

#### Motor Control Logic (Core 1, Task 7):
Priority order:
1. Emergency stop (either test mode or Core 0 command)
2. Test mode motor commands (TEST_MOTOR mode)
3. Normal autonomous mode commands (from Core 0)

#### New TCP Commands:
```
SET_MODE <mode>              - Set test mode (AUTO/TEST_MOTOR/TEST_SENSOR/CALIBRATE_IR/CALIBRATE_TOF)
GET_MODE                     - Get current test mode
TEST_MOTOR <left> <right>    - Set motor PWM (-100 to +100)
STOP_MOTOR                   - Emergency stop motors
GET_MOTOR                    - Get motor status
TEST_SENSOR <type> <index>   - Test individual sensor (IR/TOF)
threshold,<index>,<value>    - Set IR threshold (existing)
```

#### JSON Telemetry Stream (Enhanced):
```json
{
  "schema": "2.0",
  "ts": <timestamp>,
  "ir": { "raw": [...], "volts": [...] },
  "tof": { "dist": [...], "valid": [...] },
  "mode": "RUN|TEST|UNKNOWN",
  "test_mode": "AUTO|TEST_MOTOR|TEST_SENSOR|CALIBRATE_IR|CALIBRATE_TOF",
  "motors": {
    "left": -100 to 100,
    "right": -100 to 100,
    "estop": true/false,
    "timeout_sec": <seconds>
  }
}
```

#### Core 0 Motor Speeds (Autonomous Mode):
Updated to use -100 to +100 range:
- SEARCH_OPPONENT: left=50, right=-50 (rotate)
- ATTACK_FORWARD: left=100, right=100 (full speed)
- RETREAT_AND_TURN: left=-60, right=-40 (back & turn)
- EMERGENCY_REVERSE: left=-100, right=-100 (full reverse)

---

### 2. **Python GUI (viewer.py)**

#### New UI Section: Test Mode
Located between "Connection" and "Sensors" sections:
- **Mode Selector Dropdown:** Choose from AUTO, TEST_MOTOR, TEST_SENSOR, CALIBRATE_IR, CALIBRATE_TOF
- **Apply Mode Button:** Send SET_MODE command to firmware
- **Current Mode Display:** Shows active test mode from telemetry

#### Motor Control Updates:
- **Range changed:** -255 to +255 → **-100 to +100** (matches firmware duty cycle)
- **Command format:** Now sends `TEST_MOTOR <left> <right>` via TCP
- **Status display:** Shows motor values and emergency stop status from telemetry

#### Enhanced TelemetryPacket:
```python
@dataclass
class TelemetryPacket:
    # ... existing fields ...
    test_mode: str = "AUTO"       # NEW
    motor_left: int = 0           # NEW
    motor_right: int = 0          # NEW
    motor_estop: bool = False     # NEW
```

#### Real-time Display Updates:
- Test mode status updates automatically from telemetry stream
- Motor PWM values displayed with emergency stop indicator
- Mode selector syncs with firmware's current mode

---

## 🔧 Hardware Wiring

### Motor Connections:
```
Left Motor:
  - PWM: GP6
  - DIR: GP7

Right Motor:
  - PWM: GP8
  - DIR: GP9
```

### ToF Sensors (unchanged):
```
XSHUT Pins: GP12, GP11, GP13, GP10, GP14
I2C: Wire1 (SDA=GP26, SCL=GP27)
Addresses: 0x30-0x34
```

### IR Sensors (unchanged):
```
I2C: Wire (default pins)
ADS1115 Address: 0x48
```

---

## 📋 Testing Checklist

### Firmware Testing:
- [ ] Motors initialize correctly on startup
- [ ] Motors stop on emergency conditions
- [ ] TEST_MOTOR command works (verify PWM output)
- [ ] SET_MODE command switches modes correctly
- [ ] ToF sensors work with 50ms timing
- [ ] Telemetry includes test_mode and motors fields
- [ ] Cycle time is ~370ms (check Serial output)

### GUI Testing:
- [ ] Test Mode section displays correctly
- [ ] Mode selector shows all 5 modes
- [ ] Apply Mode button sends command and updates status
- [ ] Motor sliders range from -100 to +100
- [ ] Motor control sends TEST_MOTOR commands
- [ ] Status updates show current mode from telemetry
- [ ] Motor PWM values display in real-time

### Integration Testing:
- [ ] Switch to TEST_MOTOR mode from GUI
- [ ] Control motors with sliders
- [ ] Verify motor responds to commands
- [ ] Switch back to AUTO mode
- [ ] Verify autonomous control takes over
- [ ] Test emergency stop functionality

---

## 🐛 Known Issues / Notes

1. **Arduino IDE Include Errors:** VS Code shows include errors for Arduino libraries - this is normal and will compile fine in Arduino IDE.

2. **Test Mode Safety:** Motors have a 5-second timeout in TEST_MOTOR mode - if no commands received, motors automatically stop.

3. **Core 1 Cycle Time:** Increased from ~265ms to ~370ms due to longer ToF timing. This is acceptable for stable long-range detection.

4. **Motor Priority:** In TEST_MOTOR mode, GUI motor commands override Core 0 autonomous commands.

5. **Telemetry Frequency:** Reduced from ~3.77Hz to ~2.7Hz due to longer cycle time. Still sufficient for monitoring and control.

---

## 📚 File Modifications

### Modified Files:
1. `BottleSumo_TimeSliced.ino` - Main firmware file
2. `viewer.py` - Python GUI

### Unchanged Helper Files:
- `Motor.h` / `Motor.cpp` - Motor control class
- `TestModeCommon.h` - Test mode definitions
- `MotorTestMode.h` - Motor test handlers
- `SensorTestMode.h` - Sensor test handlers
- `ToFArray.h` - ToF sensor array manager
- `Ads1115Sampler.h` - IR sensor sampling
- `ButtonManager.h` - Button input handling

---

## 🎯 Next Steps

1. **Upload firmware** to Raspberry Pi Pico W
2. **Run viewer.py** and connect to robot
3. **Test motor control** in TEST_MOTOR mode
4. **Calibrate sensors** using CALIBRATE_IR/CALIBRATE_TOF modes
5. **Test autonomous mode** (AUTO) with new motor speeds
6. **Fine-tune motor speeds** in `executeAction()` if needed

---

## 📞 Support

For issues or questions:
- Check Arduino Serial Monitor for debug output
- Verify motor wiring matches pin assignments
- Ensure Python dependencies are installed: `tkinter`, `socket`, `json`
- Review telemetry JSON stream for unexpected values

---

**Integration completed successfully! 🎉**

*Generated: October 16, 2025*
