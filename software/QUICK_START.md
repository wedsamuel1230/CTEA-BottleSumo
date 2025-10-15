# Quick Start Guide - BottleSumo Motor & Test Mode

## 🚀 Getting Started

### 1. Upload Firmware
1. Open `BottleSumo_TimeSliced.ino` in Arduino IDE
2. Select board: "Raspberry Pi Pico W"
3. Upload to Pico W
4. Open Serial Monitor (115200 baud) to verify initialization

### 2. Start GUI
```bash
cd software/gui
python viewer.py
```

### 3. Connect to Robot
1. Connect your computer to robot's WiFi AP: **BottleSumo_Robot**
2. Password: **sumo2025**
3. In GUI, click **Connect** button
4. Default connection: `192.168.42.1:4242`

---

## 🎮 Using Test Mode

### Switching Modes

1. **Select mode** from dropdown:
   - `AUTO` - Normal autonomous operation
   - `TEST_MOTOR` - Manual motor control
   - `TEST_SENSOR` - Individual sensor testing
   - `CALIBRATE_IR` - IR sensor calibration
   - `CALIBRATE_TOF` - ToF sensor calibration

2. **Click "Apply Mode"** button

3. **Check status** - "Current: MODE" updates when confirmed

### AUTO Mode (Default)
- Robot runs autonomous sumo logic
- Core 0 makes decisions based on sensors
- Motors controlled by state machine

### TEST_MOTOR Mode
**Purpose:** Test motors individually before competition

1. Set mode to `TEST_MOTOR`
2. Check "Enable Transmission" checkbox
3. Move sliders to control motors:
   - Range: -100 to +100 (% duty cycle)
   - Negative = reverse, Positive = forward
4. Motor status shows current PWM values
5. ⚠️ **Safety:** Motors stop after 5 seconds without commands

**Tips:**
- Test left motor: Left=50, Right=0
- Test right motor: Left=0, Right=50
- Test forward: Both=50
- Test rotation: Left=50, Right=-50

### TEST_SENSOR Mode
**Purpose:** Verify sensor readings

1. Set mode to `TEST_SENSOR`
2. Send command via TCP:
   ```
   TEST_SENSOR IR 0    # Test IR sensor 0
   TEST_SENSOR IR -1   # Test all IR sensors
   TEST_SENSOR TOF 1   # Test ToF sensor 1 (front)
   TEST_SENSOR TOF -1  # Test all ToF sensors
   ```
3. Watch GUI for focused sensor readings

### CALIBRATE_IR Mode
**Purpose:** Find optimal edge detection thresholds

1. Set mode to `CALIBRATE_IR`
2. Send command: `CALIBRATE_IR START`
3. Move robot over:
   - White paper (edge of ring)
   - Black mat (inside ring)
4. Send command: `CALIBRATE_IR STOP`
5. Firmware suggests threshold values
6. Apply thresholds in GUI's "Threshold Configuration"

### CALIBRATE_TOF Mode
**Purpose:** Test ToF distance accuracy

1. Set mode to `CALIBRATE_TOF`
2. Send command: `CALIBRATE_TOF START`
3. Place objects at varying distances (30mm - 1500mm)
4. Send command: `CALIBRATE_TOF STOP`
5. Check min/max distance readings

---

## 🎯 Competition Workflow

### Pre-Match Checklist
```
1. ✅ Upload firmware
2. ✅ Connect to robot WiFi
3. ✅ Start GUI and connect
4. ✅ Set mode to TEST_MOTOR
5. ✅ Test both motors forward/reverse
6. ✅ Test rotation (opposite directions)
7. ✅ Verify IR sensors detect edge
8. ✅ Verify ToF sensors detect opponent
9. ✅ Set mode to AUTO
10. ✅ Press RUN MODE button on robot
11. ✅ Place on ring
```

### During Match
- Robot operates autonomously in AUTO mode
- Monitor telemetry in GUI:
  - IR voltages (edge detection)
  - ToF distances (opponent detection)
  - Robot state (action/danger level)
  - Motor commands

### Post-Match Analysis
1. Review telemetry data
2. Adjust thresholds if needed
3. Tune motor speeds in code if necessary

---

## 📊 GUI Layout

```
┌─────────────────────────────────────────┐
│ Connection                              │
│ [Host] [Port] [Connect]                 │
├─────────────────────────────────────────┤
│ 🆕 Test Mode                            │
│ [Mode Dropdown] [Apply] Current: AUTO   │
├─────────────────────────────────────────┤
│ Sensors (IR)                            │
│ [Sensor 0] Raw | Voltage | ▓▓▓░░       │
│ [Sensor 1] Raw | Voltage | ▓▓░░░       │
│ [Sensor 2] Raw | Voltage | ▓░░░░       │
│ [Sensor 3] Raw | Voltage | ▓▓▓▓▓       │
├─────────────────────────────────────────┤
│ Motor Control                           │
│ Motor 1: [===========|===========]  50  │
│ Motor 2: [===========|===========] -30  │
│ ☑ Enable Transmission                   │
│ Status: ✓ Active | M1:50 M2:-30        │
├─────────────────────────────────────────┤
│ Threshold Configuration                 │
│ Sensor 0: [2.50] [Apply]                │
│ Sensor 1: [2.50] [Apply]                │
│ ...                                     │
├─────────────────────────────────────────┤
│ ToF Sensors                             │
│ Right: 1200mm ✓ Valid  ▓▓▓░░           │
│ Front:  500mm ✓ Valid  ▓▓▓▓▓           │
│ Left:   800mm ✓ Valid  ▓▓▓▓░           │
├─────────────────────────────────────────┤
│ Robot State                             │
│ Action: ATTACK_FORWARD                  │
│ Edge Detected: False                    │
├─────────────────────────────────────────┤
│ System Info                             │
│ Timestamp: 123456                       │
│ Core0 freq: 100Hz                       │
│ Core1 freq: 2.7Hz                       │
└─────────────────────────────────────────┘
```

---

## 🔧 Troubleshooting

### Motors not responding
1. Check mode is `TEST_MOTOR`
2. Verify "Enable Transmission" is checked
3. Check motor wiring (PWM/DIR pins)
4. Verify PWM values are non-zero
5. Check Serial Monitor for errors

### GUI not connecting
1. Verify WiFi connection to robot's AP
2. Check IP address (should be 192.168.42.1)
3. Verify port 4242
4. Check firmware is running (Serial Monitor)
5. Try disconnecting and reconnecting

### Sensors reading incorrectly
1. Enter CALIBRATE_IR or CALIBRATE_TOF mode
2. Follow calibration procedure
3. Adjust thresholds in GUI
4. Verify sensor wiring and I2C addresses

### Mode not changing
1. Ensure robot is connected
2. Check "Apply Mode" button is clicked
3. Verify telemetry shows correct mode
4. Check Serial Monitor for mode change confirmation

### Emergency stop won't clear
1. Set mode to TEST_MOTOR
2. Send `STOP_MOTOR` command
3. Wait 5 seconds for timeout
4. Send new `TEST_MOTOR` command

---

## 🎛️ Advanced Commands

### Manual TCP Commands
Connect via telnet or Python socket:
```bash
telnet 192.168.42.1 4242
```

Then send commands:
```
SET_MODE TEST_MOTOR
TEST_MOTOR 50 50
GET_MOTOR
STOP_MOTOR
```

### Python Example
```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.42.1', 4242))

# Set motor speeds
sock.sendall(b"TEST_MOTOR 50 -50\n")
response = sock.recv(1024)
print(response.decode())

sock.close()
```

---

## 📈 Performance Monitoring

### Core 1 Cycle Time
- Target: ~370ms (~2.7Hz)
- Check Serial Monitor: "Core 1 cycle: XXms"
- Warnings shown if tasks exceed budget

### Core 0 Loop Rate
- Target: ~100Hz (10ms loop)
- Monitors sensor data freshness
- Adjusts motor commands in real-time

### Task Budget Warnings
```
⚠️ ADS1 overrun: 15ms (budget: 10ms)
⚠️ ToF overrun: 280ms (budget: 250ms)
⚠️ Motor PWM overrun: 8ms
⚠️ WiFi overrun: 65ms (budget: 50ms)
```
- If frequent, reduce task complexity
- Check sensor I2C bus speed
- Reduce WiFi client count

---

## 💡 Tips & Best Practices

1. **Always test motors** in TEST_MOTOR mode before competition
2. **Calibrate sensors** in competition environment (lighting, surface)
3. **Monitor cycle time** - should be consistent ~370ms
4. **Use AUTO mode** only when ready for autonomous operation
5. **Keep safety timeout** enabled in test modes
6. **Backup threshold values** before calibrating
7. **Check WiFi signal strength** in Robot State panel
8. **Review telemetry** after each match for optimization

---

## 📞 Support & Debugging

### Serial Monitor Output
```
Core 1: I/O Hub Initializing...
✓ ADS1115 ready
✓ ToF sensors: 5/5 online
✓ Buttons ready
✓ Motors ready
✓ WiFi AP: BottleSumo_Robot @ 192.168.42.1
✓ TCP server on port 4242
Core 1: Ready. Starting time-sliced scheduler...
```

### Common Error Messages
- `ERROR: ADS1115 init failed` - Check I2C wiring
- `ERROR: ToF config failed` - Check Wire1 connections
- `⚠️ Motor test timeout` - No commands received in 5s
- `ERROR: TEST_MOTOR requires 2 arguments` - Invalid command format

### Getting Help
1. Check Serial Monitor for detailed logs
2. Review INTEGRATION_SUMMARY.md
3. Check ARCHITECTURE_DIAGRAM.md for system design
4. Verify hardware connections match pin assignments

---

**Happy Testing! 🏆**

*Quick Start Guide v1.0 - October 16, 2025*
