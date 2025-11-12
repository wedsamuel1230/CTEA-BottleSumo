# Pico W Car Tracker - Telemetry Viewer

Simple Python GUI application for real-time visualization of car tracker telemetry data.

## Features

- **Real-time telemetry display** (10Hz updates)
- **Live plots** for distance, direction bias, speed, and motor speeds
- **Sensor array visualization** (5 ToF sensors)
- **Connection management** with auto-status updates
- **History tracking** (last 100 samples)

## Requirements

- Python 3.7 or higher
- tkinter (built-in with Python)
- matplotlib (for plotting)

## Installation

1. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

   Or install matplotlib directly:
   ```bash
   pip install matplotlib
   ```

2. **Connect to Pico W WiFi:**
   - SSID: `PicoW-CarTracker`
   - Password: `tracking123`
   - The Pico W will assign an IP address automatically

## Usage

1. **Connect to Pico W WiFi network** (see above)

2. **Run the GUI:**
   ```bash
   python3 telemetry_viewer.py
   ```

3. **Connect to telemetry server:**
   - Default host: `192.168.4.1`
   - Default port: `8080`
   - Click "Connect" button

4. **View real-time data:**
   - Left panel: Current telemetry values
   - Right panel: Live plots with history

## GUI Layout

### Left Panel (Current Values)
- **Distance** - Closest detected object (mm)
- **Direction Bias** - Steering direction (-1=right, 0=center, +1=left)
- **Speed** - Calculated motor speed
- **Left/Right Motors** - Individual motor speeds
- **Sensor Array** - 5 ToF sensor readings (R45, R23, M0, L23, L45)
- **Timestamp** - millis() from Pico W

### Right Panel (Plots)
- **Distance Plot** - Object distance over time
- **Bias Plot** - Direction bias over time
- **Speed Plot** - Motor speed over time
- **Motor Speeds** - Left (blue) and right (red) motor speeds

## Troubleshooting

### Cannot connect
- Verify WiFi connection to `PicoW-CarTracker`
- Check that Pico W is powered and running `car_tracking.ino`
- Verify IP address (should be `192.168.4.1` for AP mode)
- Try pinging: `ping 192.168.4.1`

### No data updates
- Check Serial Monitor on Arduino IDE for Pico W status
- Verify sensors are online in Pico W output
- Try disconnecting and reconnecting

### Plots not updating
- matplotlib may need to be updated: `pip install --upgrade matplotlib`
- Check Python version: `python3 --version` (needs 3.7+)

## Alternative: Command-Line Testing

Test TCP connection without GUI using netcat:
```bash
nc 192.168.4.1 8080
```

Or with Python:
```python
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('192.168.4.1', 8080))
while True:
    data = s.recv(1024)
    if data:
        print(data.decode('utf-8'), end='')
```

## JSON Format

Telemetry stream format:
```json
{
  "dist": 495,
  "bias": 1.00,
  "speed": 47.1,
  "left": 18.8,
  "right": 47.1,
  "R45": 0,
  "R23": 0,
  "M0": 0,
  "L23": 0,
  "L45": 495,
  "timestamp": 12345
}
```

## License

MIT License - Free to use and modify
