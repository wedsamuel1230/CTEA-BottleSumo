# TCP Motor Control for Bottle Sumo Robot

Arduino-based TCP motor control system for Raspberry Pi Pico W with dual motor control, edge detection sensors, and safety features.

## Features

- **WiFi Access Point**: Creates `BottleSumo_AP` for wireless control
- **TCP JSON Lines Protocol**: Simple text-based command interface
- **Dual Motor Control**: Independent left/right motor control via existing Motor library
- **Edge Detection**: 4x QRE1113 sensors via ADS1115 I2C ADC
- **Safety System**: Automatic retreat patterns and emergency stop
- **Session Authentication**: Token-based access control

## Hardware Requirements

- Raspberry Pi Pico W (RP2040 + WiFi)
- 2x Brushless motor drivers (DIR + PWM interface)
- 4x QRE1113 reflectance sensors
- 1x ADS1115 16-bit ADC (I2C)
- Power supply (appropriate for motors and Pico)

## GPIO Pinout

| Function | Pin | Notes |
|----------|-----|-------|
| Left Motor PWM | GP11 | 20 kHz PWM |
| Left Motor DIR | GP10 | Digital output |
| Right Motor PWM | GP14 | 20 kHz PWM |
| Right Motor DIR | GP13 | Digital output |
| I2C SDA (Wire1) | GP26 | ADS1115 connection |
| I2C SCL (Wire1) | GP27 | ADS1115 connection |

### Sensor Channel Mapping

- **A0**: Front-left sensor
- **A1**: Front-right sensor
- **A2**: Rear-left sensor
- **A3**: Rear-right sensor

## Arduino Setup

### Required Libraries

Install these libraries via Arduino Library Manager:

1. **Arduino-Pico** (Board Support Package)
   - Board Manager URL: `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
   - Select: `Tools > Board > Raspberry Pi RP2040 Boards > Raspberry Pi Pico W`

2. **ArduinoJson** (by Benoit Blanchon)
   - Version 6.x or later

3. **Adafruit ADS1X15** (by Adafruit)
   - For ADS1115 ADC support

### Motor Library Setup

The project uses the existing `Motor.h` and `Motor.cpp` library located in:
```
TcpMotorControl/libraries/CTEA_Motor/src/
```

Arduino IDE automatically includes libraries in the sketch folder's `libraries/` directory.

### Build and Upload

1. Open `TcpMotorControl.ino` in Arduino IDE
2. Select board: `Tools > Board > Raspberry Pi Pico W`
3. Configure settings:
   - Flash Size: `2MB (no FS)`
   - CPU Speed: `133 MHz`
   - Optimize: `Optimize Even More (-O3)`
4. Connect Pico W via USB
5. Select port: `Tools > Port > COMx` (Windows) or `/dev/ttyACMx` (Linux/Mac)
6. Click **Upload**

## Configuration

Edit `TcpMotorControl.ino` to customize:

```cpp
// WiFi AP Configuration
const char* AP_SSID = "BottleSumo_AP";
const char* AP_PASSWORD = "sumobot123456";
const uint16_t TCP_PORT = 5000;

// Authentication token (min 12 chars)
const char* AUTH_TOKEN = "BottleSumo2025Secure";
```

## Usage

### 1. Connect to WiFi AP

- **SSID**: `BottleSumo_AP`
- **Password**: `sumobot123456`
- **IP Address**: `192.168.4.1` (default AP IP)
- **TCP Port**: `5000`

### 2. Establish TCP Connection

Using netcat (Linux/Mac):
```bash
nc 192.168.4.1 5000
```

Using telnet (Windows):
```cmd
telnet 192.168.4.1 5000
```

Or use the **GUI Test Client** (recommended):
```bash
python test_client_gui.py
```

Or use a Python script:
```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("192.168.4.1", 5000))

# Send command
sock.sendall(b'{"action":"auth","token":"BottleSumo2025Secure"}\n')
response = sock.recv(1024)
print(response.decode())
```

### 2b. Using the GUI Test Client

The GUI client (`test_client_gui.py`) provides a user-friendly interface:

**Features:**
- Visual connection status indicator
- Real-time motor control with sliders
- Quick action buttons (forward, reverse, turn, stop)
- Emergency stop with confirmation
- Live status monitoring with auto-refresh
- Sensor calibration interface
- Command and response logging

**To launch:**
```bash
python test_client_gui.py
```

**GUI Layout:**
- **Connection Panel**: Enter IP/port and connect
- **Authentication Panel**: Enter token and authenticate
- **Motor Control**: Use sliders or quick buttons
- **Status Monitor**: Real-time robot state
- **Sensor Calibration**: Auto-calibration wizard
- **Command Log**: See all communication

### 3. Authenticate

Before any motor commands, authenticate:

```json
{"action":"auth","token":"BottleSumo2025Secure"}
```

Response:
```json
{"status":"ok"}
```

### 4. Control Motors

#### Set motor speeds
Values range from -255 (full reverse) to +255 (full forward):

```json
{"action":"set","left":200,"right":200}
```

Forward motion:
```json
{"action":"set","left":150,"right":150}
```

Reverse motion:
```json
{"action":"set","left":-150,"right":-150}
```

Turn right (differential):
```json
{"action":"set","left":200,"right":100}
```

#### Stop motors
```json
{"action":"stop"}
```

#### Emergency stop
```json
{"action":"estop"}
```

> **Note**: Emergency stop latches the system into safe mode. Manual intervention (power cycle or clear command) required to resume.

### 5. Status Query

Get current system state:

```json
{"action":"status"}
```

Response:
```json
{
  "status":"ok",
  "auth":true,
  "motors":{"left":0,"right":0},
  "sensors":{
    "raw":[15234,15891,15123,15456],
    "flags":[false,false,false,false]
  },
  "safety":{"estop":false,"latched":false}
}
```

### 6. Calibrate Sensors

**Before first use**, calibrate sensors on a white surface:

```json
{"action":"calibrate","mode":"auto","samples":64}
```

The system will sample the white surface and set thresholds automatically.

## Safety Features

### Automatic Retreat Patterns

When edge sensors detect the ring boundary, the robot automatically retreats:

| Sensors Triggered | Retreat Action |
|------------------|----------------|
| Front-left only | Backward-right |
| Front-right only | Backward-left |
| Both front | Straight backward |
| Rear-left only | Forward-right |
| Rear-right only | Forward-left |
| Both rear | Straight forward |
| Left side | Retreat right |
| Right side | Retreat left |

### Emergency Stop Conditions

The system triggers emergency stop when:
- **3 or more sensors** detect edges simultaneously
- **estop command** is received via TCP
- **Client timeout** (5 seconds without activity)
- **Client disconnect** during active motion

### Motion Validation

Commands that would drive the robot toward a detected edge are automatically blocked.

## Error Codes

| Code | Description |
|------|-------------|
| `AUTH_REQUIRED` | Must authenticate before control |
| `AUTH_INVALID` | Incorrect token |
| `AUTH_RATE_LIMIT` | Too many failed attempts (60s cooldown) |
| `BAD_REQUEST` | Malformed JSON or missing fields |
| `OUT_OF_RANGE` | Motor values outside [-255, 255] |
| `UNSAFE_STATE` | Motion blocked by safety system |
| `INTERNAL_ERROR` | System error |

## Serial Monitor Output

Connect via USB at **115200 baud** to see diagnostic messages:

```
========================================
TCP Motor Control - Initializing
========================================

[Setup] Initializing motors...
[MotorController] Initialized L/R motors
[Setup] Initializing ADS1115 sensors...
[Sensors] ADS1115 initialized on Wire1
[Setup] Starting WiFi AP: BottleSumo_AP
[Setup] AP IP: 192.168.4.1
[Setup] TCP server listening on port 5000

[Setup] Initialization complete!
========================================

[Heartbeat] Auth=N Motors=0,0 Sensors=0x0 Safe=Y
[WiFi] Client connected from 192.168.4.2
[Command] Received: {"action":"auth","token":"BottleSumo2025Secure"}
[Command] AUTH successful
[Heartbeat] Auth=Y Motors=0,0 Sensors=0x0 Safe=Y
```

## Troubleshooting

### Motors don't move
- Check power supply to motor drivers
- Verify GPIO connections (PWM and DIR pins)
- Check DIR pin logic levels match driver expectations
- Use serial monitor to confirm commands are received

### Sensors always triggered
- Run calibration command on white surface
- Check I2C connections (GP26/GP27)
- Verify ADS1115 address (default 0x48)
- Use status command to check raw sensor values

### Cannot connect to WiFi AP
- Verify AP password (min 8 characters for WPA2)
- Check client device WiFi settings
- Confirm Pico W has stable power
- Monitor serial output for AP startup errors

### Authentication fails
- Ensure token matches configured value
- Token must be minimum 12 characters
- Check for extra whitespace in command JSON
- Wait 60 seconds if rate-limited

## Advanced Configuration

### Timing Adjustments

Edit timing constants in `TcpMotorControl.ino`:

```cpp
const uint32_t SENSOR_UPDATE_INTERVAL_MS = 20;    // Sensor sample rate
const uint32_t SAFETY_CHECK_INTERVAL_MS = 10;     // Safety loop rate
const uint32_t CLIENT_TIMEOUT_MS = 5000;          // Inactivity timeout
```

### Sensor Threshold Tuning

Manually set thresholds (if auto-calibration insufficient):

```cpp
sensors.setAllThresholds(15000, 15200, 14800, 15100);
```

Values are 16-bit (0-32767). Lower values = darker surface (edge detected).

## Architecture

The system uses a **non-blocking cooperative scheduler** with millis() timing:

- **Task 1**: Handle TCP client I/O
- **Task 2**: Update sensors (50 Hz)
- **Task 3**: Safety evaluation (100 Hz)
- **Task 4**: Heartbeat logging (1 Hz)
- **Task 5**: Timeout monitoring

No `delay()` calls in main loop ensures responsive safety system.

## License

MIT License - See LICENSE file for details.

## References

- [Specification](../specs/1-tcp-motor-control/spec.md)
- [Implementation Plan](../specs/1-tcp-motor-control/plan.md)
- [Command Contracts](../specs/1-tcp-motor-control/contracts/jsonl-commands.md)
- [Constitution](.specify/memory/constitution.md)
