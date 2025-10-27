# Quickstart: TCP Motor Control (Pico W)

## Prerequisites
- Arduino IDE installed
- Board Manager: Raspberry Pi Pico/RP2040 by Earle Philhower (install Pico W support)
- Libraries:
  - ArduinoJson
  - Adafruit_ADS1X15 (ADS1115 compatible)
- Existing `Motor.h` and `Motor.cpp` available in repo (placed under lib/Motor or adjust include paths)

## Wiring (proposed)
- Left PWM: GP11 → Driver PWM L
- Left DIR:  GP10 → Driver DIR L
- Right PWM: GP14 → Driver PWM R
- Right DIR: GP13 → Driver DIR R
- I2C1 (Wire1) SDA:  GP26 → ADS1115 SDA
- I2C1 (Wire1) SCL:  GP27 → ADS1115 SCL
- ADS1115 VCC 3V3, GND
- QRE1113 outputs → ADS1115 A0..A3

## Build
1. Open the sketch (src/main.ino) in Arduino IDE
2. Select Board: Raspberry Pi Pico W
3. Verify/Upload

## Use
1. Robot powers on and starts Wi‑Fi AP (SSID in code)
2. Connect from a laptop/phone; open a TCP client to the server port
3. Send AUTH, then commands as JSON Lines, e.g.:
```
{"action":"auth","token":"YOURTOKEN"}
{"action":"set","left":150,"right":150}
{"action":"stop"}
{"action":"status"}
```

## Calibration
- Run `{ "action":"calibrate","mode":"auto","samples":64 }`
- Verify thresholds and sensor flags via `status`

## Safety
- Software estop command or detected edge forces motors off immediately (≤50 ms typical)
- On disconnect/inactivity, motors go safe‑off
