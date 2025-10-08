# BottleSumo Dual Core

This Arduino sketch demonstrates dual-core functionality on the Raspberry Pi Pico W for the BottleSumo robot.

## Architecture

### Core 0 (Main Core)
- **Purpose**: Real-time motor control and sensor reading
- **Functions**: `setup()` and `loop()`
- **Responsibilities**:
  - Motor control (forward, backward, left, right, stop)
  - Line sensor reading
  - Emergency stop handling
  - Fast response to sensor inputs

### Core 1 (Second Core)
- **Purpose**: WiFi and TCP server operations
- **Functions**: `setup1()` and `loop1()`
- **Responsibilities**:
  - WiFi connection management
  - TCP server for remote control
  - Command parsing and handling
  - Network communication

## Hardware Connections

### Motor Pins
- LEFT_MOTOR_FWD: GPIO 16
- LEFT_MOTOR_REV: GPIO 17
- RIGHT_MOTOR_FWD: GPIO 18
- RIGHT_MOTOR_REV: GPIO 19

### Sensor Pins
- LINE_SENSOR_1: GPIO 26
- LINE_SENSOR_2: GPIO 27
- LINE_SENSOR_3: GPIO 28

## WiFi Configuration

Update the following lines with your WiFi credentials:
```cpp
const char* ssid = "YourNetworkSSID";
const char* password = "YourPassword";
```

## TCP Commands

Connect to the robot's IP address on port 80 and send the following commands:

- `FORWARD` - Move forward at speed 200
- `BACKWARD` - Move backward at speed 200
- `LEFT` - Turn left at speed 150
- `RIGHT` - Turn right at speed 150
- `STOP` - Stop all motors
- `EMERGENCY` - Emergency stop (also stops motors)
- `RESET` - Reset emergency stop state
- `STATUS` - Get current robot status

## Usage

1. Update WiFi credentials in the code
2. Upload to Raspberry Pi Pico W
3. Open Serial Monitor at 115200 baud
4. Note the IP address displayed after WiFi connection
5. Connect via TCP client (e.g., telnet, netcat) to control the robot

Example using netcat:
```bash
nc <robot-ip-address> 80
FORWARD
STOP
STATUS
```

## Benefits of Dual-Core Design

- **Core separation**: Network operations don't block motor control
- **Reliability**: Motor control maintains consistent timing even during WiFi reconnection
- **Safety**: Emergency stop on Core 0 can respond immediately
- **Performance**: Both cores run in parallel for better responsiveness
