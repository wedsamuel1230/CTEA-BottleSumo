# Edge Detection System - Directional Retreat Logic

## 📍 Sensor Physical Layout

The BottleSumo robot has 4x QRE1113 IR sensors for edge detection:

```
        FRONT (Top)
    ┌─────────────────┐
    │  A0         A1  │  ← Top sensors
    │   ●           ●  │
    │                 │
    │     ROBOT       │
    │                 │
    │   ●           ●  │
    │  A2         A3  │  ← Bottom sensors
    └─────────────────┘
        REAR (Bottom)
```

### Sensor Mapping:
- **A0** (ADS1115 Channel 0): **Top-Left** (front-left corner)
- **A1** (ADS1115 Channel 1): **Top-Right** (front-right corner)
- **A2** (ADS1115 Channel 2): **Bottom-Left** (rear-left corner)
- **A3** (ADS1115 Channel 3): **Bottom-Right** (rear-right corner)

---

## 🧠 Edge Detection Logic

### Detection Method
- Each sensor outputs voltage: **low voltage = white surface**, **high voltage = edge/void**
- Threshold-based detection: `sensor_voltage > threshold` → edge detected
- Default threshold: **2.5V** (configurable per sensor via TCP command)

### Priority System
The robot's decision-making follows this priority order:

1. **🚨 HIGHEST PRIORITY: Edge Detection** - Immediate retreat from detected edge
2. **⚔️ MEDIUM PRIORITY: Opponent Detection** - Attack if ToF sensors detect opponent
3. **🔍 LOWEST PRIORITY: Search Mode** - Rotate to find opponent if no edges or opponent

---

## 🎯 Directional Retreat Actions

### Expanded SumoAction Enum

The enum now includes 10 states (expanded from original 5):

```cpp
enum SumoAction {
  SEARCH_OPPONENT,          // Default: rotate to find opponent
  ATTACK_FORWARD,           // Attack: full speed toward opponent
  
  // Edge Detection Retreat Actions (8 directional states)
  RETREAT_FORWARD,          // Both bottom (A2+A3) → move forward
  RETREAT_BACKWARD,         // Both top (A0+A1) → move backward
  RETREAT_FORWARD_LEFT,     // Bottom-right (A3) → diagonal forward-left
  RETREAT_FORWARD_RIGHT,    // Bottom-left (A2) → diagonal forward-right
  RETREAT_BACKWARD_LEFT,    // Top-right (A1) → diagonal backward-left
  RETREAT_BACKWARD_RIGHT,   // Top-left (A0) → diagonal backward-right
  RETREAT_LEFT,             // Right side (A1+A3) → spin left
  RETREAT_RIGHT,            // Left side (A0+A2) → spin right
  
  EMERGENCY_STOP,           // 3+ sensors or critical failure
  IDLE                      // Test mode or stopped
};
```

---

## 🔢 Edge Pattern Detection (Bitfield System)

### Bitfield Encoding
Pattern is a 4-bit value: `[A3 A2 A1 A0]`

- Bit 0 = A0 (top-left)
- Bit 1 = A1 (top-right)
- Bit 2 = A2 (bottom-left)
- Bit 3 = A3 (bottom-right)

### Pattern Matching Table

| Pattern (Binary) | Hex | Sensors Detected | Action | Motor Speeds (L, R) |
|-----------------|-----|------------------|--------|---------------------|
| `0b0001` | 0x1 | A0 only | RETREAT_BACKWARD_RIGHT | (-80, -40) |
| `0b0010` | 0x2 | A1 only | RETREAT_BACKWARD_LEFT | (-40, -80) |
| `0b0100` | 0x4 | A2 only | RETREAT_FORWARD_RIGHT | (80, 40) |
| `0b1000` | 0x8 | A3 only | RETREAT_FORWARD_LEFT | (40, 80) |
| `0b0011` | 0x3 | A0+A1 (top) | RETREAT_BACKWARD | (-80, -80) |
| `0b1100` | 0xC | A2+A3 (bottom) | RETREAT_FORWARD | (80, 80) |
| `0b0101` | 0x5 | A0+A2 (left side) | RETREAT_RIGHT | (60, -60) |
| `0b1010` | 0xA | A1+A3 (right side) | RETREAT_LEFT | (-60, 60) |
| `0b1001` | 0x9 | A0+A3 (diagonal) | RETREAT_FORWARD_RIGHT | (80, 40) |
| `0b0110` | 0x6 | A1+A2 (diagonal) | RETREAT_FORWARD_LEFT | (40, 80) |

**Emergency Stop:** 3 or more sensors (patterns with 3+ bits set) → `EMERGENCY_STOP`

---

## 🚗 Motor Command Strategy

### Motor Speed Range
- **-100 to +100** (PWM duty cycle percentage)
- Negative = reverse, Positive = forward

### Retreat Strategies

#### 1. **Straight Retreat** (both sensors on same edge)
```cpp
RETREAT_FORWARD:  L=80, R=80    // Both bottom sensors
RETREAT_BACKWARD: L=-80, R=-80  // Both top sensors
```

#### 2. **Diagonal Retreat** (single corner sensor)
- Asymmetric speeds for simultaneous retreat + turn
- Faster motor on side away from edge

```cpp
RETREAT_FORWARD_RIGHT: L=80, R=40   // Bottom-left sensor (A2)
RETREAT_FORWARD_LEFT:  L=40, R=80   // Bottom-right sensor (A3)
```

#### 3. **Spin Retreat** (side sensors)
- Counter-rotation for rapid direction change
- One motor forward, one reverse

```cpp
RETREAT_LEFT:  L=-60, R=60   // Right side sensors
RETREAT_RIGHT: L=60, R=-60   // Left side sensors
```

---

## 📊 Telemetry & Monitoring

### JSON Stream Output

The robot streams edge detection data via WiFi TCP (port 4242):

```json
{
  "schema": "2.0",
  "ts": 123456,
  "ir": {
    "raw": [1234, 2345, 3456, 4567],
    "volts": [1.500, 2.800, 1.200, 3.100]
  },
  "robot_state": {
    "action": "RETREAT_FORWARD_RIGHT",
    "edges": {
      "count": 1,
      "pattern": 4,
      "A0": false,
      "A1": false,
      "A2": true,
      "A3": false
    }
  },
  "motors": {
    "left": 80,
    "right": 40
  }
}
```

### Serial Console Logging

Core 0 logs edge detection state every 100 loops (~1 second):

```
Core 0: Action=RETREAT_FORWARD_RIGHT | Edges=1 [A0:0 A1:0 A2:1 A3:0] | Mode=1 | IR0=1.50V | ToF0=1200mm | Motors L:80 R:40
```

---

## 🧪 Testing & Calibration

### 1. **Threshold Calibration**

Test each sensor on white surface vs. edge:

```bash
# Via TCP commands (connect to 192.168.4.1:4242)
threshold,0,2.5   # Set A0 threshold to 2.5V
threshold,1,2.5   # Set A1 threshold to 2.5V
threshold,2,2.5   # Set A2 threshold to 2.5V
threshold,3,2.5   # Set A3 threshold to 2.5V
```

Watch telemetry for `ir.volts` values:
- **White surface:** ~0.5-1.5V
- **Edge/void:** ~3.0-4.5V

Adjust thresholds to midpoint between these values.

### 2. **Edge Detection Test Scenarios**

| Test Case | Sensors Triggered | Expected Action | Expected Motors |
|-----------|------------------|-----------------|-----------------|
| Front-left corner | A0 | RETREAT_BACKWARD_RIGHT | L=-80, R=-40 |
| Front-right corner | A1 | RETREAT_BACKWARD_LEFT | L=-40, R=-80 |
| Rear-left corner | A2 | RETREAT_FORWARD_RIGHT | L=80, R=40 |
| Rear-right corner | A3 | RETREAT_FORWARD_LEFT | L=40, R=80 |
| Front edge | A0+A1 | RETREAT_BACKWARD | L=-80, R=-80 |
| Rear edge | A2+A3 | RETREAT_FORWARD | L=80, R=80 |
| Left side | A0+A2 | RETREAT_RIGHT | L=60, R=-60 |
| Right side | A1+A3 | RETREAT_LEFT | L=-60, R=60 |

### 3. **Manual Test Mode**

Use viewer.py to observe sensor voltages in real-time:
1. Connect to robot WiFi: `BottleSumo_Robot`
2. Launch viewer.py
3. Connect to `192.168.4.1:4242`
4. Hold robot over edge and observe IR sensor readings

---

## 🛠️ Code Architecture

### Key Functions

#### `analyzeEdges(sensors)`
- Input: `SensorData` struct
- Output: `EdgeDetection` struct with bitfield pattern
- Purpose: Convert sensor voltages to edge detection state

#### `decideAction(sensors)`
- Input: `SensorData` struct
- Output: `SumoAction` enum
- Priority logic:
  1. Check edge detection (highest priority)
  2. Check opponent detection (ToF sensors)
  3. Default to search mode

#### `executeAction(action, sensors)`
- Input: `SumoAction` enum, `SensorData` struct
- Output: `MotorCommand` struct
- Purpose: Convert action to motor PWM commands

#### `actionToString(action)`
- Input: `SumoAction` enum
- Output: String representation for telemetry/debugging

---

## 🔍 Debugging Tips

### Problem: Robot doesn't detect edges
1. Check IR sensor voltages in telemetry
2. Lower threshold if sensors read high on white surface
3. Verify sensors are not obstructed or dirty

### Problem: False edge detections
1. Check for ambient light interference
2. Raise threshold if sensors read too high normally
3. Test on different surface (black tape on white background recommended)

### Problem: Robot retreats in wrong direction
1. Verify sensor physical layout matches code mapping
2. Check motor wiring (swap if reversed)
3. Verify motor speed signs (negative = reverse)

### Problem: Multiple sensors trigger constantly
1. Check for sensor hardware failure
2. Verify I2C communication with ADS1115
3. Check for ground loops or power supply noise

---

## 📚 References

- **QRE1113 Datasheet:** Reflective IR sensor specifications
- **ADS1115 Library:** Adafruit_ADS1X15.h for I2C ADC
- **Motor Control:** Motor.h/.cpp for PWM management
- **Architecture:** See ARCHITECTURE_DIAGRAM.md for system overview

---

*Document Version: 1.0*  
*Last Updated: October 18, 2025*  
*Author: AI Agent (Autonomous Principal Engineer)*
