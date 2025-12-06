# Edge Detection Testing & Validation Guide

## 🎯 Testing Objectives

Verify that the new directional edge detection system:
1. ✅ Correctly detects edges on all 4 sensors
2. ✅ Retreats in the appropriate direction for each sensor pattern
3. ✅ Maintains priority: Edge > Opponent > Search
4. ✅ Reports accurate telemetry data

---

## 📋 Pre-Test Checklist

### Hardware Setup
- [ ] Raspberry Pi Pico W installed on robot
- [ ] 4x QRE1113 IR sensors connected to ADS1115 (channels A0-A3)
- [ ] 5x VL53L0X ToF sensors on Wire1 I2C bus
- [ ] 2x DC motors connected (GP6/7 left, GP8/9 right)
- [ ] Button on GP16 set to RUN mode
- [ ] Power supply connected and charged
- [ ] USB cable for serial monitoring (optional)

### Software Setup
- [✅] Arduino IDE 2.x installed
- [✅] Raspberry Pi Pico/RP2040 board support installed
- [✅] Required libraries:
  - Adafruit_ADS1X15
  - Adafruit_VL53L0X
  - WiFi (built-in for Pico W)
- [✅] BottleSumo_TimeSliced.ino compiles without errors

### Testing Environment
- [ ] Sumo ring or elevated platform (1m+ diameter recommended)
- [ ] Black edge marking (electrical tape or painted border)
- [ ] White center surface (contrast with edge)
- [ ] Laptop/phone for WiFi connection
- [ ] viewer.py running for telemetry monitoring

---

## 🧪 Test Procedure

### Test 1: Sensor Calibration

**Objective:** Verify each IR sensor can distinguish white surface from edge.

#### Steps:
1. Upload firmware to Pico W
2. Open Serial Monitor (115200 baud)
3. Wait for Core 0 and Core 1 ready messages
4. Connect to WiFi AP: `BottleSumo_Robot` (password: `sumo2025`)
5. Launch viewer.py and connect to `192.168.4.1:4242`
6. Place robot in center of platform (all sensors on white)
7. Record baseline voltages for A0-A3 (should be ~0.5-1.5V)
8. Hold robot over edge so A0 is off platform
9. Record edge voltage for A0 (should be ~3.0-4.5V)
10. Repeat for A1, A2, A3

#### Expected Results:
```
White Surface: A0=1.2V, A1=1.3V, A2=1.1V, A3=1.4V
Edge/Void:     A0=3.8V, A1=3.9V, A2=3.7V, A3=4.0V
```

#### Calibration:
If threshold (default 2.5V) is not between white and edge voltages, adjust via TCP:
```bash
threshold,0,2.5  # Adjust A0 threshold
threshold,1,2.5  # Adjust A1 threshold
threshold,2,2.5  # Adjust A2 threshold
threshold,3,2.5  # Adjust A3 threshold
```

---

### Test 2: Single Sensor Edge Detection

**Objective:** Verify correct retreat direction for each individual sensor.

#### Test Matrix:

| Test # | Sensor | Edge Location | Expected Action     | Expected Motors     | Expected Direction   |
|--------|--------|---------------|-----------------    |-----------------|-------------------   |
| 2.1 | A0 | Front-left corner | RETREAT_BACKWARD_RIGHT | L=-80, R=-40    | Back + Turn Right    |
| 2.2 | A1 | Front-right corner | RETREAT_BACKWARD_LEFT | L=-40, R=-80    | Back + Turn Left     |
| 2.3 | A2 | Rear-left corner | RETREAT_FORWARD_RIGHT   | L=80, R=40      | Forward + Turn Right |
| 2.4 | A3 | Rear-right corner | RETREAT_FORWARD_LEFT   | L=40, R=80      | Forward + Turn Left  |

#### Steps (for each test):
1. Place robot on platform with specified sensor near edge
2. Set button to RUN mode (GP16)
3. Observe robot response
4. Check telemetry JSON for correct action string
5. Verify motor speeds match expected values
6. Confirm robot moves away from edge

#### Serial Output Example:
```
Core 0: Action=RETREAT_BACKWARD_RIGHT | Edges=1 [A0:1 A1:0 A2:0 A3:0] | Mode=1 | IR0=3.80V | ToF0=1200mm | Motors L:-80 R:-40
```

#### JSON Telemetry Example:
```json
{
  "robot_state": {
    "action": "RETREAT_BACKWARD_RIGHT",
    "edges": {
      "count": 1,
      "pattern": 1,
      "A0": true,
      "A1": false,
      "A2": false,
      "A3": false
    }
  },
  "motors": {
    "left": -80,
    "right": -40
  }
}
```

---

### Test 3: Dual Sensor Edge Detection (Straight Edges)

**Objective:** Verify correct retreat for front/back/side edges.

#### Test Matrix:

| Test # | Sensors | Edge Type | Expected Action | Expected Motors | Expected Direction |
|--------|---------|-----------|-----------------|-----------------|-------------------|
| 3.1 | A0+A1 | Front edge | RETREAT_BACKWARD | L=-80, R=-80 | Straight Back |
| 3.2 | A2+A3 | Rear edge | RETREAT_FORWARD | L=80, R=80 | Straight Forward |
| 3.3 | A0+A2 | Left side | RETREAT_RIGHT | L=60, R=-60 | Spin Right |
| 3.4 | A1+A3 | Right side | RETREAT_LEFT | L=-60, R=60 | Spin Left |

#### Steps (for each test):
1. Position robot so specified sensors are over edge
2. Verify edge detection in telemetry (`"count": 2`)
3. Confirm correct action and motor speeds
4. Observe robot retreat behavior

---

### Test 4: Emergency Stop (3+ Sensors)

**Objective:** Verify emergency stop when robot is falling off platform.

#### Steps:
1. Lift robot and hold it so 3 sensors are off platform
2. Verify action = `EMERGENCY_STOP`
3. Confirm motors stop (`emergency_stop: true`)
4. Check telemetry shows `"count": 3` or higher

#### Expected Output:
```
Core 0: Action=EMERGENCY_STOP | Edges=3 [A0:1 A1:1 A2:1 A3:0] | Mode=1 | ...
```

---

### Test 5: Priority System

**Objective:** Verify edge detection overrides opponent detection.

#### Steps:
1. Place robot on platform with no edges nearby
2. Place object in front of robot (trigger ToF sensor)
3. Verify action = `ATTACK_FORWARD`
4. Slowly move robot toward edge while object remains in front
5. When edge sensor triggers, verify action changes to retreat
6. Confirm edge detection has higher priority

#### Expected Behavior:
- **No edge, opponent present:** `ATTACK_FORWARD`
- **Edge detected, opponent present:** `RETREAT_*` (edge takes priority)

---

### Test 6: Search Mode (No Edge, No Opponent)

**Objective:** Verify robot searches when no stimuli present.

#### Steps:
1. Place robot in center of platform
2. Ensure no objects within ToF range (~800mm)
3. Verify action = `SEARCH_OPPONENT`
4. Confirm motors rotate in place (L=50, R=-50)

---

### Test 7: Full Sumo Match Simulation

**Objective:** End-to-end integration test of all behaviors.

#### Steps:
1. Place robot on sumo ring
2. Set button to RUN mode
3. Place opponent robot or object nearby
4. Observe complete behavior sequence:
   - Robot searches (rotates)
   - Detects opponent (attacks)
   - Approaches edge (retreats)
   - Repositions and continues
5. Monitor telemetry for state transitions
6. Verify smooth transitions between states

#### Success Criteria:
- ✅ Robot never falls off platform
- ✅ Robot attacks when opponent detected
- ✅ Robot retreats appropriately from all edges
- ✅ No crashes or unexpected stops
- ✅ Telemetry data is consistent with behavior

---

## 📊 Telemetry Monitoring

### Key Metrics to Watch

#### IR Sensor Voltages
```json
"ir": {
  "volts": [1.200, 1.300, 1.100, 1.400]  // A0-A3
}
```
- **White:** ~1-2V
- **Edge:** ~3-4V

#### Edge Detection State
```json
"robot_state": {
  "edges": {
    "count": 1,        // Number of sensors triggered
    "pattern": 4,      // Bitfield: [A3 A2 A1 A0]
    "A0": false,
    "A1": false,
    "A2": true,        // This sensor detected edge
    "A3": false
  }
}
```

#### Action State
```json
"robot_state": {
  "action": "RETREAT_FORWARD_RIGHT"  // Current action
}
```

#### Motor Commands
```json
"motors": {
  "left": 80,
  "right": 40,
  "estop": false
}
```

---

## 🐛 Troubleshooting

### Problem: Sensor always reads high voltage

**Symptoms:**
- IR voltage always ~3-4V even on white surface
- False edge detections

**Causes:**
- Sensor obstructed or dirty
- Poor I2C connection to ADS1115
- Ambient light interference

**Solutions:**
1. Clean sensor lens with isopropyl alcohol
2. Check I2C wiring (SDA/SCL)
3. Shield sensors from direct sunlight
4. Lower threshold value

---

### Problem: Robot doesn't retreat from edge

**Symptoms:**
- Edge detected in telemetry but no motor response
- Action shows retreat but motors don't move

**Causes:**
- Mode button not in RUN position
- Motor wiring issue
- Motor PWM pins misconfigured

**Solutions:**
1. Verify button on GP16 is pressed (RUN mode)
2. Check motor connections (GP6/7 left, GP8/9 right)
3. Test motors in TEST_MOTOR mode via viewer.py
4. Verify Core 1 Task 7 is running (motor PWM update)

---

### Problem: Wrong retreat direction

**Symptoms:**
- Robot retreats toward edge instead of away
- Motor directions reversed

**Causes:**
- Motor wiring swapped
- Sensor position mismapped in code
- Motor direction pins inverted

**Solutions:**
1. Swap motor wire polarity
2. Verify sensor physical positions match code
3. Check `Motor.cpp` direction logic
4. Test individual motors in test mode

---

### Problem: Jerky or unstable movement

**Symptoms:**
- Robot vibrates during retreat
- Unsmooth direction changes

**Causes:**
- Motor PWM frequency too low
- Time budget overruns in Core 1
- Mechanical binding

**Solutions:**
1. Verify PWM frequency = 20kHz
2. Check Core 1 timing logs for overruns
3. Check for mechanical obstructions
4. Reduce motor speeds in `executeAction()`

---

## ✅ Test Completion Checklist

### Functional Tests
- [ ] Test 1: Sensor calibration complete
- [ ] Test 2.1: A0 (front-left) retreat verified
- [ ] Test 2.2: A1 (front-right) retreat verified
- [ ] Test 2.3: A2 (rear-left) retreat verified
- [ ] Test 2.4: A3 (rear-right) retreat verified
- [ ] Test 3.1: Front edge retreat verified
- [ ] Test 3.2: Rear edge retreat verified
- [ ] Test 3.3: Left side retreat verified
- [ ] Test 3.4: Right side retreat verified
- [ ] Test 4: Emergency stop verified
- [ ] Test 5: Priority system verified
- [ ] Test 6: Search mode verified
- [ ] Test 7: Full match simulation passed

### Documentation
- [ ] Serial logs captured
- [ ] Telemetry JSON samples saved
- [ ] Threshold values documented
- [ ] Any issues/bugs noted

### Performance
- [ ] Core 0 loop frequency ~100Hz
- [ ] Core 1 cycle time ~370ms
- [ ] No time budget overruns
- [ ] WiFi telemetry streaming stable
- [ ] Motor response immediate (<50ms)

---

## 📝 Test Report Template

```markdown
# Edge Detection Test Report

**Date:** [Date]
**Tester:** [Name]
**Firmware Version:** BottleSumo_TimeSliced v2.0

## Configuration
- Thresholds: A0=2.5V, A1=2.5V, A2=2.5V, A3=2.5V
- Platform: [Diameter, Material]
- Edge Marking: [Type, Color]

## Test Results

### Single Sensor Tests
- A0: [PASS/FAIL] - [Notes]
- A1: [PASS/FAIL] - [Notes]
- A2: [PASS/FAIL] - [Notes]
- A3: [PASS/FAIL] - [Notes]

### Dual Sensor Tests
- Front (A0+A1): [PASS/FAIL] - [Notes]
- Rear (A2+A3): [PASS/FAIL] - [Notes]
- Left (A0+A2): [PASS/FAIL] - [Notes]
- Right (A1+A3): [PASS/FAIL] - [Notes]

### Priority & Integration
- Emergency Stop: [PASS/FAIL] - [Notes]
- Priority System: [PASS/FAIL] - [Notes]
- Search Mode: [PASS/FAIL] - [Notes]
- Full Match: [PASS/FAIL] - [Notes]

## Issues Found
1. [Issue description]
2. [Issue description]

## Performance Metrics
- Core 0 freq: [Hz]
- Core 1 cycle: [ms]
- Telemetry rate: [Hz]
- Motor latency: [ms]

## Recommendations
- [Recommendation 1]
- [Recommendation 2]

## Conclusion
[Overall assessment: PASS/FAIL with summary]
```

---

*Test Guide Version: 1.0*  
*Compatible with: BottleSumo_TimeSliced v2.0+*  
*Last Updated: October 18, 2025*
