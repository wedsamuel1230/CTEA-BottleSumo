# Motor API Documentation

## Overview

The `Motor` class provides a hardware abstraction for controlling brushless DC motors using DIR (direction) + PWM (pulse-width modulation) signaling on Raspberry Pi Pico W (RP2040).

## Class: Motor

### Constructor

```cpp
Motor(uint pwm_pin, uint dir_pin)
```

Creates a new Motor instance.

**Parameters:**
- `pwm_pin`: GPIO pin for PWM output (must support PWM function)
- `dir_pin`: GPIO pin for direction control (digital output)

**Example:**
```cpp
Motor leftMotor(11, 10);  // PWM on GP11, DIR on GP10
Motor rightMotor(14, 13); // PWM on GP14, DIR on GP13
```

### Methods

#### begin()

```cpp
void begin(uint32_t freq)
```

Initialize the motor with specified PWM frequency.

**Parameters:**
- `freq`: PWM frequency in Hz (recommended: 10000-25000 Hz)

**Example:**
```cpp
motor.begin(20000); // 20 kHz PWM
```

**Note:** Must be called before any motor control commands.

#### setDuty()

```cpp
void setDuty(float speed)
```

Set motor speed and direction.

**Parameters:**
- `speed`: Duty cycle from -100.0 to +100.0
  - Positive values: Forward direction (DIR pin HIGH)
  - Negative values: Reverse direction (DIR pin LOW)
  - Zero: Motor off (0% duty, DIR pin LOW)
  - Values >100 or <-100 are saturated to ±100

**Example:**
```cpp
motor.setDuty(75.0);   // 75% forward
motor.setDuty(-50.0);  // 50% reverse
motor.setDuty(0.0);    // Stop
```

**Internal Behavior:**
- Converts duty to 0-100% range (absolute value)
- Sets DIR pin based on sign
- Calculates PWM level based on configured frequency
- Updates hardware PWM channel immediately

#### stop()

```cpp
void stop()
```

Stop the motor immediately.

**Example:**
```cpp
motor.stop();
```

**Effect:**
- Sets PWM duty to 0%
- Sets DIR pin LOW
- Motor coasts to stop (no braking)

## Hardware Interface

### PWM Output

The Motor class uses RP2040 hardware PWM:
- **Clock**: 125 MHz system clock
- **Resolution**: 16-bit (65536 levels)
- **Frequency**: Configurable (10-25 kHz typical)
- **Slices**: Automatically assigned based on GPIO pin

### Direction Control

- **HIGH (1)**: Forward direction
- **LOW (0)**: Reverse direction

Ensure driver board logic matches this convention. Some drivers may require:
- Inverted logic (swap in driver configuration)
- Additional enable pin (implement separately if needed)

## Integration Notes

### BottleSumo Motor Controller Wrapper

The `MotorController` class wraps two `Motor` instances:

```cpp
class MotorController {
  Motor _motorLeft;   // Left motor
  Motor _motorRight;  // Right motor
  
  // Maps [-255, 255] command values to [-100, 100] duty
  float mapValueToDuty(int16_t value);
};
```

**Value Mapping:**
- Command range: [-255, 255] (matches common robot APIs)
- Motor duty range: [-100, 100] (percentage)
- Conversion: `duty = value * (100.0 / 255.0)`

### Pin Assignment

Per `plan.md` and hardware map:

| Motor | PWM Pin | DIR Pin |
|-------|---------|---------|
| Left  | GP11    | GP10    |
| Right | GP14    | GP13    |

### Safety Considerations

1. **Initialize before use**: Always call `begin()` before `setDuty()`
2. **Power-on state**: Motors start in undefined state; call `stop()` after `begin()`
3. **Emergency stop**: Call `stop()` from main loop safety override
4. **Series resistors**: Use 1-4.7 kΩ resistors on signal lines if needed
5. **Logic level matching**: Verify driver expects 3.3V logic (RP2040 native)

## Example Usage

### Basic Initialization and Control

```cpp
#include "Motor.h"

Motor leftMotor(11, 10);
Motor rightMotor(14, 13);

void setup() {
  Serial.begin(115200);
  
  leftMotor.begin(20000);
  rightMotor.begin(20000);
  
  // Ensure motors start stopped
  leftMotor.stop();
  rightMotor.stop();
  
  Serial.println("Motors initialized");
}

void loop() {
  // Forward at 50%
  leftMotor.setDuty(50.0);
  rightMotor.setDuty(50.0);
  delay(2000);
  
  // Stop
  leftMotor.stop();
  rightMotor.stop();
  delay(1000);
  
  // Reverse at 75%
  leftMotor.setDuty(-75.0);
  rightMotor.setDuty(-75.0);
  delay(2000);
  
  // Stop
  leftMotor.stop();
  rightMotor.stop();
  delay(1000);
}
```

### Non-Blocking Control

```cpp
unsigned long lastUpdateMs = 0;
float currentDuty = 0.0;

void loop() {
  unsigned long now = millis();
  
  if (now - lastUpdateMs >= 50) { // 20 Hz update
    lastUpdateMs = now;
    
    // Ramp duty cycle
    currentDuty += 1.0;
    if (currentDuty > 100.0) currentDuty = -100.0;
    
    leftMotor.setDuty(currentDuty);
    rightMotor.setDuty(currentDuty);
  }
  
  // Other tasks...
}
```

## Troubleshooting

### Motor doesn't spin

1. Check power supply to motor driver
2. Verify GPIO pin assignments match hardware
3. Confirm PWM frequency is appropriate for driver
4. Check DIR pin polarity matches driver expectations
5. Measure PWM signal with oscilloscope/logic analyzer

### Motor spins wrong direction

Swap DIR pin logic in driver board or invert in code:

```cpp
// In Motor.cpp setDuty():
dir = (speed > 0) ? 0 : 1; // Inverted logic
```

Or physically swap motor wires.

### Unstable speed control

1. Increase PWM frequency (15-25 kHz)
2. Add capacitor across motor terminals (0.1-1 µF)
3. Use shielded cables for PWM/DIR signals
4. Check for noise on power supply

## References

- [RP2040 Datasheet - PWM Chapter](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [Arduino-Pico PWM Documentation](https://arduino-pico.readthedocs.io/en/latest/pwm.html)
- Project specification: `specs/1-tcp-motor-control/spec.md`
- GPIO wiring: `docs/gpio_wiring.md`
