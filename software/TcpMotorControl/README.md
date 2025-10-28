# TcpMotorControl (Arduino Pico W)

AP-mode TCP server for safe motor control over JSON Lines using existing Motor.h/Motor.cpp.

- Board: Raspberry Pi Pico W (earlephilhower/arduino-pico)
- Motors: DIR+PWM (Left: LPWM GP11/LDIR GP10, Right: RPWM GP14/RDIR GP13)
- Protocol: JSON Lines over TCP, port 3333
- Auth: Optional session token via `auth` (>=12 chars). If set, all commands require auth.
- Safety: Inactivity safe-off (1s without command -> stop). `estop` stops and clears auth.

## Files
- `TcpMotorControl.ino` — WiFi AP + TCP loop, safe-off
- `motor_controller.h/.cpp` — wrapper around 2x Motor instances
- `command_parser.h/.cpp` — naive JSONL parsing (no ArduinoJson dependency)
- `libraries/CTEA_Motor/src/Motor.*` — Local copy of existing Motor implementation so Arduino builds the library

## Build
1. Open folder `TcpMotorControl` in Arduino IDE (as a separate sketch).
2. Select board: Raspberry Pi Pico W
3. Compile and upload.

If you build from VS Code with Arduino extension, ensure this sketch folder is the workspace root.

## Try it
After flashing, the Pico W starts an AP:
- SSID: `BottleSumo-TCP`
- Password: `sumo123456`
- Port: `3333`

Connect and `telnet 192.168.4.1 3333`, then send one command per line:

- `{"cmd":"ping"}`
- `{"cmd":"auth","token":"your-very-long-token"}`
- `{"cmd":"drive","left":30,"right":30}`
- `{"cmd":"stop"}`
- `{"cmd":"estop"}`
- `{"cmd":"status"}`

Responses are single JSON objects per line.

## Notes
- This is a minimal, dependency-free parser. For robustness, consider ArduinoJson later.
- Sensors and advanced safety patterns can be integrated in future phases.
