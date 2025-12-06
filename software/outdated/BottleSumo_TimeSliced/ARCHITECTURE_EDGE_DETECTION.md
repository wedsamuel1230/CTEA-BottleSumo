# Edge Detection System - Architecture & Flow Diagrams

## 🏗️ System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     RASPBERRY PI PICO W (RP2040)                        │
│                                                                           │
│  ┌──────────────────────────────┐  ┌──────────────────────────────────┐ │
│  │      CORE 0: State Machine   │  │      CORE 1: I/O Hub             │ │
│  │      (~100Hz)                │  │      (~2.7Hz cycles)             │ │
│  │                              │  │                                  │ │
│  │  1. Read shared sensor data  │  │  1. ADS1115 IR read (10ms)      │ │
│  │  2. analyzeEdges()           │◄─┼──2. ToF array read (250ms)      │ │
│  │  3. decideAction()           │  │  3. ADS1115 IR read (10ms)      │ │
│  │     - Edge detection         │  │  4. Button sample (5ms)         │ │
│  │     - Opponent detection     │  │  5. Button debounce (20ms)      │ │
│  │     - Search mode            │  │  6. ADS1115 IR read (10ms)      │ │
│  │  4. executeAction()          │  │  7. Motor PWM update (5ms) ◄────┼─┐│
│  │  5. Write motor commands ────┼─►│  8. WiFi/TCP handle (50ms)     │ ││
│  │                              │  │  9. ADS1115 IR read (10ms)      │ ││
│  └──────────────────────────────┘  └──────────────────────────────────┘ ││
│                                                                           ││
└───────────────────────────────────────────────────────────────────────────┘│
                                                                             │
┌─────────────────────────────────────────────────────────────────────────┐│
│                          HARDWARE INTERFACES                            ││
│                                                                         ││
│  ┌──────────────────┐  ┌──────────────────┐  ┌────────────────────┐  ││
│  │  ADS1115 (Wire)  │  │  VL53L0X (Wire1) │  │  Motors (PWM)      │  ││
│  │  I2C @ 0x48      │  │  I2C @ 0x30-0x34 │  │  GP6/7 (Left)      │◄─┘│
│  │                  │  │                  │  │  GP8/9 (Right)     │    │
│  │  A0: Top-Left    │  │  5x ToF sensors  │  │  20kHz PWM freq    │    │
│  │  A1: Top-Right   │  │  (opponent       │  │  -100 to +100 duty │    │
│  │  A2: Bottom-Left │  │   detection)     │  │                    │    │
│  │  A3: Bottom-Right│  │                  │  │                    │    │
│  └──────────────────┘  └──────────────────┘  └────────────────────┘    │
│                                                                          │
│  ┌──────────────────┐  ┌──────────────────┐                            │
│  │  WiFi AP         │  │  Buttons         │                            │
│  │  BottleSumo_Robot│  │  GP15: Test Mode │                            │
│  │  TCP Port 4242   │  │  GP16: Run Mode  │                            │
│  └──────────────────┘  └──────────────────┘                            │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 🧠 Edge Detection Decision Flow

```
                        ┌─────────────────────┐
                        │  Core 0 Loop Entry  │
                        │   (~10ms period)    │
                        └──────────┬──────────┘
                                   │
                        ┌──────────▼──────────┐
                        │ Read Sensor Data    │
                        │ (mutex protected)   │
                        └──────────┬──────────┘
                                   │
                        ┌──────────▼──────────┐
                        │  analyzeEdges()     │
                        │  - Check A0-A3      │
                        │  - Build bitfield   │
                        │  - Count edges      │
                        └──────────┬──────────┘
                                   │
                        ┌──────────▼──────────┐
                        │  decideAction()     │
                        │  PRIORITY SYSTEM    │
                        └──────────┬──────────┘
                                   │
              ┌────────────────────┼────────────────────┐
              │                    │                    │
    ┌─────────▼─────────┐ ┌────────▼────────┐ ┌───────▼────────┐
    │ Priority 1:       │ │ Priority 2:     │ │ Priority 3:    │
    │ EDGE DETECTION    │ │ OPPONENT        │ │ SEARCH MODE    │
    │ (HIGHEST)         │ │ DETECTION       │ │ (DEFAULT)      │
    └─────────┬─────────┘ └────────┬────────┘ └───────┬────────┘
              │                    │                    │
    ┌─────────▼─────────┐          │                    │
    │ edges.count >= 3? │          │                    │
    └─────────┬─────────┘          │                    │
              │                    │                    │
         YES  │  NO                │                    │
    ┌─────────▼─────────┐          │                    │
    │ EMERGENCY_STOP    │          │                    │
    └───────────────────┘          │                    │
              │                    │                    │
         NO   │                    │                    │
    ┌─────────▼─────────┐          │                    │
    │ edges.count > 0?  │          │                    │
    └─────────┬─────────┘          │                    │
              │                    │                    │
         YES  │  NO                │                    │
    ┌─────────▼─────────┐          │                    │
    │ Pattern Matching: │          │                    │
    │                   │          │                    │
    │ 0b0001 → A0 only ─┼─────┐    │                    │
    │ 0b0010 → A1 only ─┼─┐   │    │                    │
    │ 0b0100 → A2 only ─┼─┤   │    │                    │
    │ 0b1000 → A3 only ─┼─┤   │    │                    │
    │ 0b0011 → A0+A1 ───┼─┤   │    │                    │
    │ 0b1100 → A2+A3 ───┼─┤   │    │                    │
    │ 0b0101 → A0+A2 ───┼─┤   │    │                    │
    │ 0b1010 → A1+A3 ───┼─┤   │    │                    │
    │ 0b1001 → Diagonal ┼─┤   │    │                    │
    │ 0b0110 → Diagonal ┼─┘   │    │                    │
    └───────────────────┘     │    │                    │
                              │    │                    │
              ┌───────────────┘    │                    │
              │                    │                    │
    ┌─────────▼─────────┐          │                    │
    │ Return Directional│          │                    │
    │ RETREAT_* action  │          │                    │
    └─────────┬─────────┘          │                    │
              │                    │                    │
              │        NO edges detected               │
              │             ┌──────▼────────┐          │
              │             │ ToF opponent? │          │
              │             └──────┬────────┘          │
              │                    │                   │
              │               YES  │  NO               │
              │         ┌──────────▼────────┐          │
              │         │ ATTACK_FORWARD    │          │
              │         └───────────────────┘          │
              │                    │                   │
              │                    │  NO opponent      │
              │                    │      ┌────────────▼─────────┐
              │                    │      │ SEARCH_OPPONENT      │
              │                    │      └──────────────────────┘
              │                    │                   │
              └────────────────────┴───────────────────┘
                                   │
                        ┌──────────▼──────────┐
                        │  executeAction()    │
                        │  Map action to      │
                        │  motor commands     │
                        └──────────┬──────────┘
                                   │
                        ┌──────────▼──────────┐
                        │ Write Motor Command │
                        │ (mutex protected)   │
                        └──────────┬──────────┘
                                   │
                        ┌──────────▼──────────┐
                        │ Core 1 reads command│
                        │ Updates motor PWM   │
                        └─────────────────────┘
```

---

## 🎯 Edge Pattern to Action Mapping

SENSOR LAYOUT:              BITFIELD PATTERN:        ACTION:
                            [A3 A2 A1 A0]

    A0    A1                0  0  0  1          RETREAT_BACKWARD_RIGHT
     ●    ○              →   Bit pattern 0x1    (Back + Turn Right)
                             
     ○    ●
    A2    A3
────────────────────────────────────────────────────────────────────

    A0    A1                0  0  1  0          RETREAT_BACKWARD_LEFT
     ○    ●              →   Bit pattern 0x2    (Back + Turn Left)
                             
     ○    ○
    A2    A3
────────────────────────────────────────────────────────────────────

    A0    A1                0  1  0  0          RETREAT_FORWARD_RIGHT
     ○    ○              →   Bit pattern 0x4    (Forward + Turn Right)
                             
     ●    ○
    A2    A3
────────────────────────────────────────────────────────────────────

    A0    A1                1  0  0  0          RETREAT_FORWARD_LEFT
     ○    ○              →   Bit pattern 0x8    (Forward + Turn Left)
                             
     ○    ●
    A2    A3
────────────────────────────────────────────────────────────────────

    A0    A1                0  0  1  1          RETREAT_BACKWARD
     ●    ●              →   Bit pattern 0x3    (Straight Back)
                             
     ○    ○
    A2    A3
────────────────────────────────────────────────────────────────────

    A0    A1                1  1  0  0          RETREAT_FORWARD
     ○    ○              →   Bit pattern 0xC    (Straight Forward)
                             
     ●    ●
    A2    A3
────────────────────────────────────────────────────────────────────

    A0    A1                0  1  0  1          RETREAT_RIGHT
     ●    ○              →   Bit pattern 0x5    (Spin Right)
                             
     ●    ○
    A2    A3
────────────────────────────────────────────────────────────────────

    A0    A1                1  0  1  0          RETREAT_LEFT
     ○    ●              →   Bit pattern 0xA    (Spin Left)
                             
     ○    ●
    A2    A3
────────────────────────────────────────────────────────────────────

KEY:  ● = Edge Detected (high voltage)
      ○ = On Platform (low voltage)
```

---

## ⚙️ Motor Command Mapping

```
┌─────────────────────────────┬──────────┬───────────┬─────────────────────┐
│ ACTION                      │ LEFT PWM │ RIGHT PWM │ PHYSICAL MOVEMENT   │
├─────────────────────────────┼──────────┼───────────┼─────────────────────┤
│ IDLE / EMERGENCY_STOP       │    0     │     0     │ Stop                │
├─────────────────────────────┼──────────┼───────────┼─────────────────────┤
│ SEARCH_OPPONENT             │   +50    │    -50    │ Rotate CCW in place │
├─────────────────────────────┼──────────┼───────────┼─────────────────────┤
│ ATTACK_FORWARD              │  +100    │   +100    │ Full speed forward  │
├─────────────────────────────┼──────────┼───────────┼─────────────────────┤
│ RETREAT_FORWARD             │   +80    │    +80    │ Forward escape      │
├─────────────────────────────┼──────────┼───────────┼─────────────────────┤
│ RETREAT_BACKWARD            │   -80    │    -80    │ Backward escape     │
├─────────────────────────────┼──────────┼───────────┼─────────────────────┤
│ RETREAT_FORWARD_LEFT        │   +40    │    +80    │ Arc forward-left    │
├─────────────────────────────┼──────────┼───────────┼─────────────────────┤
│ RETREAT_FORWARD_RIGHT       │   +80    │    +40    │ Arc forward-right   │
├─────────────────────────────┼──────────┼───────────┼─────────────────────┤
│ RETREAT_BACKWARD_LEFT       │   -40    │    -80    │ Arc backward-left   │
├─────────────────────────────┼──────────┼───────────┼─────────────────────┤
│ RETREAT_BACKWARD_RIGHT      │   -80    │    -40    │ Arc backward-right  │
├─────────────────────────────┼──────────┼───────────┼─────────────────────┤
│ RETREAT_LEFT                │   -60    │    +60    │ Spin left (CW)      │
├─────────────────────────────┼──────────┼───────────┼─────────────────────┤
│ RETREAT_RIGHT               │   +60    │    -60    │ Spin right (CCW)    │
└─────────────────────────────┴──────────┴───────────┴─────────────────────┘

PWM Range: -100 to +100
  - Negative = Reverse
  - Positive = Forward
  - Magnitude = Speed (duty cycle percentage)
  
Motor Update Rate: ~200Hz (Core 1 Task 7, 5ms budget)
```

---

## 📡 Telemetry Data Flow

```
┌────────────────────────────────────────────────────────────────┐
│                   CORE 1: buildStreamJSON()                    │
│                   (Called every ~370ms cycle)                  │
└─────────────────────┬──────────────────────────────────────────┘
                      │
          ┌───────────▼───────────┐
          │ Read Sensor Data      │
          │ (mutex protected)     │
          │ - IR voltages         │
          │ - ToF distances       │
          │ - Button mode         │
          └───────────┬───────────┘
                      │
          ┌───────────▼───────────┐
          │ Read Core 0 State     │
          │ - current_action      │
          │ - current_edges       │
          └───────────┬───────────┘
                      │
          ┌───────────▼───────────┐
          │ Build JSON String     │
          │ {                     │
          │   "ir": {...},        │
          │   "tof": {...},       │
          │   "robot_state": {    │
          │     "action": "...",  │
          │     "edges": {...}    │
          │   }                   │
          │ }                     │
          └───────────┬───────────┘
                      │
          ┌───────────▼───────────┐
          │ Stream to WiFi Clients│
          │ TCP Port 4242         │
          └───────────┬───────────┘
                      │
          ┌───────────▼───────────┐
          │ viewer.py / Terminal  │
          │ Real-time Display     │
          └───────────────────────┘
```

### JSON Schema v2.0 with Edge Detection

```json
{
  "schema": "2.0",
  "ts": 123456,
  "ir": {
    "raw": [1234, 2345, 3456, 4567],
    "volts": [1.200, 3.800, 1.100, 1.400]
  },
  "tof": {
    "dist": [1200, 500, 800, 1500, 1000],
    "valid": [true, true, true, false, true]
  },
  "mode": "RUN",
  "test_mode": "AUTO",
  "motors": {
    "left": -80,
    "right": -40,
    "estop": false
  },
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
  }
}
```

---

## 🔄 State Transition Diagram

```
                    ┌──────────────┐
                    │     IDLE     │
                    │ (Mode != RUN)│
                    └──────┬───────┘
                           │ Mode = RUN
                           ▼
                    ┌──────────────┐
            ┌──────►│ SEARCH_      │◄──────┐
            │       │  OPPONENT    │       │
            │       └──────┬───────┘       │
            │              │               │
            │ No edge,     │ ToF detects   │ No edge,
            │ no opponent  │ opponent      │ no opponent
            │              ▼               │
            │       ┌──────────────┐       │
            │       │  ATTACK_     │       │
            │       │   FORWARD    │       │
            │       └──────┬───────┘       │
            │              │               │
            │              │ Edge detected │
            │              ▼               │
    ┌───────┴─────────────────────────────┴──────┐
    │         EDGE DETECTION RETREAT             │
    │                                            │
    │  ┌──────────────┐      ┌──────────────┐    │
    │  │ RETREAT_     │      │ RETREAT_     │    │
    │  │  FORWARD     │      │  BACKWARD    │    │
    │  └──────────────┘      └──────────────┘    │
    │                                            │
    │  ┌──────────────┐      ┌──────────────┐    │
    │  │ RETREAT_     │      │ RETREAT_     │    │
    │  │  LEFT        │      │  RIGHT       │    │
    │  └──────────────┘      └──────────────┘    │
    │                                            │
    │  ┌──────────────┐      ┌──────────────┐    │
    │  │ RETREAT_FWD_ │      │ RETREAT_BWD_ │    │
    │  │  LEFT/RIGHT  │      │  LEFT/RIGHT  │    │
    │  └──────────────┘      └──────────────┘    │
    │                                            │
    └──────────────────┬─────────────────────────┘
                       │
                       │ Edge cleared
                       │ (sensor back on platform)
                       ▼
                    ┌──────────────┐
                    │ SEARCH_      │
                    │  OPPONENT    │
                    └──────────────┘
                           │
                           │ 3+ edges detected
                           ▼
                    ┌──────────────┐
                    │ EMERGENCY_   │
                    │   STOP       │
                    └──────────────┘
```

---

## 🎮 Control Flow Summary

1. **Core 1** reads sensors at ~2.7Hz → updates `shared_sensor_data`
2. **Core 0** reads sensor data at ~100Hz → analyzes edges
3. **Edge detection** takes priority over all other behaviors
4. **Pattern matching** determines optimal retreat direction
5. **Motor commands** written to `shared_motor_cmd`
6. **Core 1 Task 7** reads motor commands → updates PWM at ~200Hz
7. **Telemetry** streams action and edge state to WiFi clients

---

*Architecture Diagram Version: 1.0*  
*Last Updated: October 18, 2025*  
*System: BottleSumo_TimeSliced v2.0*
