# BottleSumo State-Machine Sketch

`simple_car_tracking.ino` is now a fully finite-state BottleSumo controller that keeps the original RP2040 pinout while adding IR boundary protection, button-controlled arming, and bilingual documentation.

## Features
- Object-oriented FSM with four handlers: Idle, Search, Track, Emergency_Stop (中英文註解)。
- 5× VL53L0X array on Wire1 for opponent tracking + weighted bias steering.
- ADS1115 sampler monitors four IR edge sensors (A0–A3) with per-channel thresholds.
- Start button (default GP16, pull-up) debounced in software to transition Idle → Search.
- Emergency state executes a back-off + spin escape whenever any IR channel trips, and only returns to Idle after the floor is safe.

## Usage
1. Open `simple_car_tracking.ino` in Arduino IDE or PlatformIO.
2. Select **Raspberry Pi Pico W** (RP2040) and confirm the VL53L0X/ADS1115 wiring defined at the top of the sketch.
3. Ensure the start button is wired to GP16 → GND (press = LOW) or update the constants to match your hardware.
4. Upload, open the serial monitor at 115200 baud, and observe `[FSM]` logs for state transitions and IR alarms.

> ⚠️ The right motor is wired inverted. `driveDifferential()` compensates internally—do not swap motor leads unless you also adjust the code.
