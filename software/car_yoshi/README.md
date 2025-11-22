# Car Yoshi - Combined Tracker

A simplified sumo robot controller combining the best features from `car_tracking` and `car_ir` without a complex state machine.

## Features

### From car_tracking:
- **Smart search mode** - Spin in place while searching for opponent
- **Bias-based tracking** - Uses ToF sensor array to calculate target direction
- **Progressive alignment** - Coarse spin (32°/s) → Fine spin (18°/s) → Attack (100%)
- **Hold behavior** - Waits 2 seconds before giving up on lost target

### From car_ir:
- **Comprehensive edge escape** - 8 different escape patterns based on edge location
- **Multi-sensor edge detection** - 4 IR sensors cover all edges
- **Smart escape maneuvers** - Backs up, turns away, then resumes

## Priority System

Simple three-level priority (no state machine):

1. **Edge Escape** (Highest Priority)
   - If any IR sensor detects white edge → Execute escape maneuver
   - Continues escape until complete, then resumes normal operation

2. **Target Tracking** (Medium Priority)
   - If target seen → Align and attack
   - If target lost < 2s → Hold position
   - If target lost > 2s → Enter search mode

3. **Search Mode** (Lowest Priority)
   - Spin in place at 35% speed
   - Continuously scan with ToF sensors

## Hardware Configuration

### Motors
- Left: PWM=GP11, DIR=GP12
- Right: PWM=GP14, DIR=GP15
- PWM Frequency: 20kHz

### Sensors
- **ToF Array**: 5× VL53L0X on I2C (GP2=SDA, GP3=SCL)
  - R45, R23, MID, L23, L45
  - Detection range: 70mm - 1000mm
  
- **IR Sensors**: 4× via ADS1115 on same I2C bus
  - A0 = Back-Left
  - A1 = Front-Left
  - A2 = Front-Right
  - A3 = Back-Right
  - Thresholds: Front=1.5V, Back=3.0V

### Button
- Start button: GP28 (active LOW with pullup)

## Behavior Parameters

```cpp
// Search
SEARCH_SPIN_SPEED = 35.0f          // Spin speed while searching

// Tracking
ALIGN_SPIN_SPEED = 32.0f           // Coarse alignment
ALIGN_FINE_SPIN_SPEED = 18.0f      // Fine alignment
ATTACK_SPEED = 100.0f              // Full speed attack
BIAS_DEADZONE = 0.1f               // Center tolerance
LOST_HOLD_MS = 2000                // Wait time before giving up

// Edge Escape
ESCAPE_SPEED = 50.0f               // Standard escape speed
BACK_ESCAPE_SPEED = 100.0f         // Fast forward when back edge
```

## Edge Escape Patterns

| Pattern | Sensors | Behavior |
|---------|---------|----------|
| 0b0001 | Back-L | Back + turn right → Forward + turn right |
| 0b0010 | Front-L | Back → Turn right strongly |
| 0b0100 | Front-R | Back → Turn left strongly |
| 0b1000 | Back-R | Back + turn left → Forward + turn left |
| 0b0011 | Left side | Back → Strong turn right |
| 0b1100 | Right side | Back → Strong turn left |
| 0b0110 | Front | Back → Turn |
| 0b1001 | Back | Fast forward |
| Other | Multiple | Emergency stop → Resume |

## Usage

1. **Upload** `car_yoshi.ino` to Raspberry Pi Pico W
2. **Open** Serial Monitor at 115200 baud
3. **Press** start button (GP28)
4. **Watch** telemetry output

## Serial Output Format

```
[12345ms] IR: BL=1.23 FL=2.45 FR=1.89 BR=2.67 | ToF: R45=120 R23=250 M0=180 L23=95 L45=310 | Target: M0 dist=180 bias=0.00
[TRACK] ATTACK - centered
```

## Tuning Guide

### Make search faster/slower
```cpp
SEARCH_SPIN_SPEED = 50.0f;  // Increase = faster search
```

### Make alignment more precise
```cpp
BIAS_DEADZONE = 0.05f;      // Smaller = tighter centering
ALIGN_FINE_SPIN_SPEED = 12.0f; // Slower = more precise
```

### Change target hold time
```cpp
LOST_HOLD_MS = 1000;        // Shorter = faster search after loss
```

### Adjust edge sensitivity
```cpp
IR_THRESHOLD_FRONT = 2.0f;  // Higher = less sensitive
IR_THRESHOLD_BACK = 3.5f;
```

## Files

- `car_yoshi.ino` - Main program
- `Car.cpp/.h` - Motor control wrapper
- `Motor.cpp/.h` - Low-level motor driver
- `ToFArray.cpp/.h` - VL53L0X sensor array manager
- `Ads1115Sampler.cpp/.h` - ADS1115 ADC interface

## Advantages Over State Machine Approach

✅ **Simpler logic** - Easy to understand and modify  
✅ **Faster response** - No state transition overhead  
✅ **Direct control** - Priority-based decision making  
✅ **Easy debugging** - Clear execution flow  
✅ **Less code** - ~400 lines vs 800+ in state machine versions

## Comparison

| Feature | car_yoshi | CAR_final_v2 | car_tracking | car_ir |
|---------|-----------|--------------|--------------|--------|
| State Machine | ❌ No | ✅ Yes | ❌ No | ❌ No |
| Dual-Core | ❌ No | ✅ Yes | ✅ Yes | ❌ No |
| Search Mode | ✅ Yes | ✅ Yes | ✅ Yes | ✅ Yes |
| Edge Escape | ✅ Yes | ✅ Yes | ❌ No | ✅ Yes |
| Target Tracking | ✅ Yes | ✅ Yes | ✅ Yes | ❌ No |
| Complexity | Low | High | Medium | Low |

---

**Created:** 2025年11月21日  
**Based on:** car_tracking + car_ir  
**Status:** ✅ Ready for testing
