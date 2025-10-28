# GPIO Wiring Diagram

## Overview

Complete GPIO pin assignments for BottleSumo TCP Motor Control system on Raspberry Pi Pico W.

## Pin Allocation Summary

| Function | GPIO Pin | Physical Pin | Type | Notes |
|----------|----------|--------------|------|-------|
| Left Motor PWM | GP11 | Pin 15 | PWM Output | 20 kHz, 3.3V logic |
| Left Motor DIR | GP10 | Pin 14 | Digital Output | Direction control |
| Right Motor PWM | GP14 | Pin 19 | PWM Output | 20 kHz, 3.3V logic |
| Right Motor DIR | GP13 | Pin 17 | Digital Output | Direction control |
| I2C1 SDA | GP26 | Pin 31 | I2C Data | ADS1115 connection |
| I2C1 SCL | GP27 | Pin 32 | I2C Clock | ADS1115 connection |
| UART TX | GP0 | Pin 1 | Debug Serial | USB serial (115200 baud) |
| UART RX | GP1 | Pin 2 | Debug Serial | USB serial |

## Motor Driver Connections

### Left Motor (Motor 1)

```
Pico W                    Motor Driver
-------                   ------------
GP11 (PWM) ------------>  PWM/SPEED Input
GP10 (DIR) ------------>  DIR/IN1 Input
GND        ------------>  GND
```

**Notes:**
- Add 1-4.7 kΩ series resistor on PWM/DIR lines if driver is 5V logic
- Verify driver logic polarity (HIGH=forward vs LOW=forward)
- Common driver ICs: DRV8833, TB6612, L298N

### Right Motor (Motor 2)

```
Pico W                    Motor Driver
-------                   ------------
GP14 (PWM) ------------>  PWM/SPEED Input
GP13 (DIR) ------------>  DIR/IN2 Input
GND        ------------>  GND
```

## Sensor Interface (ADS1115)

### I2C Bus (Wire1)

```
Pico W                    ADS1115
-------                   -------
GP26 (SDA) <----------->  SDA
GP27 (SCL) <----------->  SCL
3.3V       ------------>  VDD
GND        ------------>  GND
```

**I2C Address:** 0x48 (default, ADDR pin to GND)

### QRE1113 Sensor Connections

```
ADS1115 Channel           Sensor Position
---------------           ---------------
A0                        Front-Left
A1                        Front-Right
A2                        Rear-Left
A3                        Rear-Right
```

Each QRE1113 sensor circuit:
```
Pico 3.3V ---> [220Ω] ---> QRE1113 Anode
                           QRE1113 Cathode ---> ADS1115 ANx
                           QRE1113 Collector ---> [10kΩ] ---> GND
```

## Power Supply Architecture

```
                    +----------------+
                    | Battery Pack   |
                    | (7.4-12V LiPo) |
                    +-------+--------+
                            |
                +-----------+-----------+
                |                       |
        +-------v-------+       +-------v--------+
        | Buck (5V/3A)  |       | Motor Driver   |
        | for Pico      |       | (VIN: 7-12V)   |
        +-------+-------+       +-------+--------+
                |                       |
                |                       +---> Motors
                |
        +-------v-------+
        | Pico W        |
        | - 5V VBUS     |
        | - 3.3V Out    |
        +-------+-------+
                |
                +---> ADS1115 (3.3V)
                +---> QRE1113 sensors (3.3V)
```

**Important:**
- Pico W requires 5V on VBUS pin or 1.8-5.5V on VSYS
- Use common ground across all modules
- Keep motor power isolated from logic ground (star ground topology)
- Add 100µF capacitor near Pico VBUS
- Add 10µF capacitor near ADS1115 VDD

## PWM Slice Assignments

RP2040 PWM architecture:

| GPIO Pin | PWM Slice | PWM Channel |
|----------|-----------|-------------|
| GP10     | 5         | A           |
| GP11     | 5         | B           |
| GP13     | 6         | B           |
| GP14     | 7         | A           |

**Note:** GP10 is DIR (digital only), but shares slice 5 with GP11 (PWM). This is fine since DIR is pure digital output.

## Signal Integrity Considerations

### For Long Wire Runs (>15 cm)

1. **Twisted pair**: Twist PWM/DIR wires with GND return
2. **Series termination**: 
   - PWM: 100Ω near Pico GPIO
   - DIR: 100Ω near Pico GPIO
3. **Pull-down on DIR**: 10kΩ resistor DIR to GND at driver side
4. **Bypass capacitors**: 0.1µF at driver inputs

### For High Noise Environments

1. Ferrite beads on motor power lines
2. RC snubber (100Ω + 0.1µF) across motor terminals
3. Separate ground plane for motors if possible
4. Shield I2C cables (connect shield to GND at one end only)

## Physical Layout Recommendations

```
+----------------------------------+
|  +-----------+                   |
|  |  Pico W   |                   |
|  |  (center) |                   |
|  +-----+-----+                   |
|        |                         |
|    +---+---+                     |
|    | I2C   |                     |
|    |ADS1115|                     |
|    +---+---+                     |
|        |                         |
|   [4x QRE1113]                   |
|   (ring perimeter)               |
|                                  |
|  +---------+      +---------+    |
|  | Left    |      | Right   |    |
|  | Motor   |      | Motor   |    |
|  | Driver  |      | Driver  |    |
|  +---------+      +---------+    |
|      |                |          |
|   [Motor]          [Motor]       |
+----------------------------------+
```

**Guidelines:**
- Keep Pico and ADS1115 close together (<10 cm)
- Route I2C away from motor power traces
- Place motor drivers near motors
- QRE1113 sensors at chassis edge, facing downward

## Connector Pinouts

### Motor Driver Connector (Recommended: JST-XH 4-pin)

| Pin | Signal | Wire Color (Suggested) |
|-----|--------|------------------------|
| 1   | PWM    | Yellow                 |
| 2   | DIR    | Blue                   |
| 3   | GND    | Black                  |
| 4   | +VIN   | Red                    |

### I2C Connector (Recommended: JST-XH 4-pin)

| Pin | Signal | Wire Color |
|-----|--------|------------|
| 1   | SDA    | Green      |
| 2   | SCL    | Yellow     |
| 3   | VDD    | Red        |
| 4   | GND    | Black      |

## Verification Checklist

Before powering on:

- [ ] All GND connections are common
- [ ] Motor power is isolated from logic until verified
- [ ] No GPIO pin drives >3.3V (Pico is NOT 5V tolerant)
- [ ] Bypass capacitors installed (100nF near Pico, ADS1115)
- [ ] Motor driver logic voltage matches Pico (3.3V) or uses level shifter
- [ ] I2C pull-up resistors present (2.2-4.7 kΩ to 3.3V) if not on ADS1115 board
- [ ] PWM frequency configured correctly (10-25 kHz)
- [ ] DIR pin polarity tested with multimeter before connecting motors
- [ ] Power supply can deliver motor stall current (2-5A typical per motor)

## Troubleshooting

### No I2C Communication

1. Verify SDA/SCL not swapped
2. Check I2C pull-ups (should see ~3.3V when idle)
3. Run I2C scanner: `Wire1.beginTransmission(0x48); Wire1.endTransmission();`
4. Verify ADS1115 address with ADDR pin configuration

### Motors Don't Respond

1. Check DIR pin polarity (measure with voltmeter during forward command)
2. Verify PWM signal with oscilloscope (should see 20 kHz square wave)
3. Confirm driver enable pin is HIGH (if driver has enable)
4. Test motor directly with bench supply

### PWM Interference on Sensors

1. Move sensor wiring away from motor power traces
2. Add 0.1µF capacitor across ADS1115 input channels
3. Reduce PWM frequency to 15 kHz
4. Use shielded cable for I2C

## References

- [Raspberry Pi Pico W Pinout](https://datasheets.raspberrypi.com/picow/PicoW-A4-Pinout.pdf)
- [RP2040 GPIO Specifications](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [ADS1115 Datasheet](https://www.ti.com/lit/ds/symlink/ads1115.pdf)
- [QRE1113 Datasheet](https://www.sparkfun.com/datasheets/Robotics/QRE1113.pdf)
- Project plan: `specs/1-tcp-motor-control/plan.md`
- Motor API: `docs/motor_api.md`
