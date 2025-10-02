# Bottle Sumo Robot - ToF Sensors

## 📁 Files Overview

### 1. `TOF_Sensors.ino` (Improved Version)
**Serial communication only** - For basic debugging and USB connection.

**Improvements Applied:**
- ✅ **Point 1**: Better status code validation (0-2 only, rejects status 4)
- ✅ **Point 2**: Retry mechanism (3 attempts) for sensor initialization
- ✅ **Point 3**: I2C scan with explanatory comment for debugging
- ✅ **Point 4**: All magic numbers replaced with named constants
- ✅ **Point 5**: Not needed - retry mechanism handles sensor failures

**Key Features:**
- Ultra-fast 20ms timing budget
- ~33 readings per second
- Min/max range validation (30mm - 2000mm)
- 3-second Serial timeout for autonomous operation

---

### 2. `TOF_Sensors_WiFi_AP.ino` (WiFi Access Point Version)
**Complete WiFi AP mode** with web server for wireless debugging.

**Additional Features:**
- 🌐 Creates WiFi Access Point: `BottleSumo_Robot` (Password: `sumo2025`)
- 📊 Web dashboard at `http://192.168.4.1`
- 🔄 Real-time sensor updates (10Hz via web, 33Hz internally)
- 📡 RESTful API endpoints for Python GUI integration

---

## 🔧 Hardware Configuration

### Pin Definitions
- **GP11** (XSHUT_1): Right sensor
- **GP12** (XSHUT_2): Front sensor
- **GP13** (XSHUT_3): Left sensor
- **GP26**: I2C SDA (Wire1)
- **GP27**: I2C SCL (Wire1)

### I2C Addresses (Runtime Assigned)
- **0x30**: Right sensor
- **0x31**: Front sensor
- **0x32**: Left sensor

---

## ⚙️ Configuration Constants

```cpp
TIMING_BUDGET_US       = 20000   // 20ms per reading (ultra-fast)
VCSEL_PRE_RANGE        = 18      // Speed optimized
VCSEL_FINAL_RANGE      = 14      // Speed optimized
DETECTION_THRESHOLD_MM = 1600    // 160cm detection range
MAX_VALID_RANGE_MM     = 2000    // Maximum sensor range
MIN_VALID_RANGE_MM     = 30      // Minimum valid range
LOOP_DELAY_MS          = 30      // ~33Hz update rate
MAX_INIT_RETRIES       = 3       // Sensor init attempts
```

**To adjust detection range**, change `DETECTION_THRESHOLD_MM` at the top of the file.

---

## 📡 WiFi AP Mode - API Reference

### Connecting to Robot
1. Connect to WiFi: **BottleSumo_Robot**
2. Password: **sumo2025**
3. Robot IP: **192.168.4.1** (automatically assigned)

### Web Dashboard
```
http://192.168.4.1/
```
- Live sensor readings
- Visual direction indicator
- Updates 10 times per second

### API Endpoints

#### GET `/data` or `/json`
Returns current sensor readings in JSON format.

**Response Example:**
```json
{
  "right": {
    "distance": 450,
    "valid": true,
    "status": 0
  },
  "front": {
    "distance": 1200,
    "valid": true,
    "status": 0
  },
  "left": {
    "distance": 0,
    "valid": false,
    "status": 4
  },
  "direction": "RIGHT",
  "timestamp": 123456
}
```

#### GET `/status`
Returns WiFi and system information.

**Response Example:**
```json
{
  "wifi_ssid": "BottleSumo_Robot",
  "ip": "192.168.4.1",
  "connected_clients": 1,
  "uptime_ms": 45230
}
```

---

## 🐍 Python GUI Integration

### Basic Example
```python
import requests
import json

# Make sure you're connected to BottleSumo_Robot WiFi
ROBOT_IP = "http://192.168.4.1"

def get_sensor_data():
    response = requests.get(f"{ROBOT_IP}/data")
    data = response.json()
    return data

# Usage
while True:
    data = get_sensor_data()
    print(f"Right: {data['right']['distance']}mm")
    print(f"Front: {data['front']['distance']}mm")
    print(f"Left: {data['left']['distance']}mm")
    print(f"Direction: {data['direction']}")
    print("-" * 40)
    time.sleep(0.1)  # 10Hz updates
```

### Tkinter GUI Example (Basic)
```python
import tkinter as tk
import requests
import json

class RobotMonitor:
    def __init__(self, root):
        self.root = root
        root.title("Bottle Sumo Monitor")
        
        # Labels
        self.right_label = tk.Label(root, text="Right: ---", font=("Arial", 16))
        self.right_label.pack()
        
        self.front_label = tk.Label(root, text="Front: ---", font=("Arial", 16))
        self.front_label.pack()
        
        self.left_label = tk.Label(root, text="Left: ---", font=("Arial", 16))
        self.left_label.pack()
        
        self.direction_label = tk.Label(root, text="CLEAR", font=("Arial", 24, "bold"))
        self.direction_label.pack()
        
        # Update loop
        self.update_data()
    
    def update_data(self):
        try:
            response = requests.get("http://192.168.4.1/data", timeout=1)
            data = response.json()
            
            # Update labels
            self.right_label.config(text=f"Right: {data['right']['distance']}mm")
            self.front_label.config(text=f"Front: {data['front']['distance']}mm")
            self.left_label.config(text=f"Left: {data['left']['distance']}mm")
            self.direction_label.config(text=data['direction'])
            
        except Exception as e:
            print(f"Error: {e}")
        
        # Schedule next update (100ms = 10Hz)
        self.root.after(100, self.update_data)

# Run GUI
root = tk.Tk()
app = RobotMonitor(root)
root.mainloop()
```

---

## 📊 Status Code Reference

| Code | Meaning | Accepted? | Description |
|------|---------|-----------|-------------|
| 0 | Valid | ✅ Yes | Good reading, high confidence |
| 1 | Sigma Fail | ✅ Yes | Slightly less accurate but usable |
| 2 | Signal Fail | ✅ Yes | Weak signal, may be valid |
| 3 | Min Range | ❌ No | Object too close (<50mm) |
| 4 | Phase Fail | ❌ No | Out of range or no object |
| 5+ | Hardware Error | ❌ No | Sensor malfunction |

**Why reject status 4?**
Status 4 often indicates "no object detected" but can return a distance value. This causes false positives where the robot thinks it sees something when it doesn't.

---

## 🔍 Troubleshooting

### Sensors fail to initialize
- Check wiring (especially XSHUT pins)
- Verify I2C connections (SDA=GP26, SCL=GP27)
- Check power supply (3.3V stable)
- Try increasing `delay()` in init sequence
- Check Serial output for which sensor failed

### WiFi AP not appearing
- Make sure you're using Pico W (not regular Pico)
- Check WiFi password is at least 8 characters
- Look for `Access Point created successfully!` in Serial
- Try resetting the Pico

### Web page loads but no data
- Check that sensors initialized successfully
- Verify `latest_reading` is being updated in loop
- Try accessing `/data` endpoint directly to see JSON

### Python script can't connect
- Verify you're connected to `BottleSumo_Robot` WiFi
- Check IP address is `192.168.4.1`
- Install required library: `pip install requests`
- Check firewall settings

---

## 🎯 Competition Tips

1. **Test before match**: Run for 5+ minutes to ensure stability
2. **Adjust threshold**: Tune `DETECTION_THRESHOLD_MM` for your ring size
3. **Backup code**: Always have serial-only version ready
4. **Battery level**: Low voltage can cause sensor failures
5. **Sensor placement**: Ensure no obstructions in sensor FOV (25° cone)

---

## 📝 Next Steps

1. ✅ Upload improved code to Pico W
2. ✅ Test sensor initialization and readings
3. ✅ Test WiFi AP mode and web dashboard
4. 🔲 Create Python Tkinter GUI for advanced monitoring
5. 🔲 Add sensor data logging for analysis
6. 🔲 Implement competition strategy logic

---

## 🤝 Support

For issues or questions:
1. Check Serial output for error messages
2. Verify I2C scan shows 3 devices
3. Test with serial-only version first
4. Review status codes for sensor validity

**Good luck in the competition! 🏆**
