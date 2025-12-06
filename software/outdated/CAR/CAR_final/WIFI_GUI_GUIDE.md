# WiFi & GUI Viewer Guide

## Overview
The CAR_final_v2 robot broadcasts telemetry data over WiFi, which can be viewed using the GUI viewer or any TCP client. This guide shows you how to connect and visualize the robot's sensor data in real-time.

## Quick Start

### 1. Power On Robot
```bash
# Robot starts automatically and creates WiFi AP:
SSID: BottleSumo_AP
Password: sumobot123456
IP Address: 192.168.4.1
TCP Port: 8080
```

### 2. Connect to Robot WiFi
On your computer:
- Open WiFi settings
- Connect to network: **BottleSumo_AP**
- Enter password: **sumobot123456**
- Wait for connection (may take 10-20 seconds)

### 3. Test Connection
```bash
# Simple test - view raw JSON stream
nc 192.168.4.1 8080

# You should see JSON like:
{"t":12345,"ir":[0.12,1.45,1.50,0.10],"tof":[120,115,110,118,125],"m":[35.0,35.0],"s":3}
```

## JSON Telemetry Format

### Current v2 Format
```json
{
  "t": 12345,                          // Timestamp (milliseconds)
  "ir": [0.12, 1.45, 1.50, 0.10],      // IR sensor voltages [A0, A1, A2, A3]
  "tof": [120, 115, 110, 118, 125],    // ToF distances (mm) [R45, R23, C, L23, L45]
  "m": [35.0, 35.0],                   // Motor speeds [left, right]
  "s": 3                               // State (0=IDLE, 2=SEARCHING, 3=TRACKING, etc.)
}
```

### Field Descriptions

#### IR Sensors (`ir`)
- **A0** (index 0): Back-Left IR sensor voltage
- **A1** (index 1): Front-Left IR sensor voltage
- **A2** (index 2): Front-Right IR sensor voltage
- **A3** (index 3): Back-Right IR sensor voltage
- **Range**: 0.0 - 4.1V (higher = closer to edge)

#### ToF Sensors (`tof`)
- **R45** (index 0): Right 45° sensor distance (mm)
- **R23** (index 1): Right 23° sensor distance (mm)
- **C** (index 2): Center 0° sensor distance (mm)
- **L23** (index 3): Left 23° sensor distance (mm)
- **L45** (index 4): Left 45° sensor distance (mm)
- **Range**: 70 - 1000mm (invalid readings show as 0)

#### Motors (`m`)
- **left**: Left motor speed (-100 to +100)
- **right**: Right motor speed (-100 to +100)
- Negative = backward, Positive = forward

#### State (`s`)
- **0**: IDLE (waiting to start)
- **1**: CALIBRATING (sensor calibration)
- **2**: SEARCHING (spinning to find target)
- **3**: TRACKING (aligning with target)
- **4**: ATTACKING (full-speed attack)
- **5**: EDGE_AVOIDING (emergency edge escape)

## Using the GUI Viewer

### Option 1: Use Existing viewer_compact.py (Needs Adaptation)

The existing GUI viewer expects a different JSON format. Let's create an adapter script:

```bash
cd /Users/wici/Documents/GitHub/CTEA-BottleSumo/software/gui
python3 adapter_v2.py
```

### Option 2: Use Simple Python Viewer (New)

I'll create a simple viewer specifically for v2:

```python
# Save as: simple_viewer_v2.py
import socket
import json
import tkinter as tk
from tkinter import ttk

class RobotViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("BottleSumo v2 Viewer")
        self.root.geometry("800x600")
        
        # Connection frame
        conn_frame = ttk.Frame(root, padding=10)
        conn_frame.pack(fill='x')
        
        ttk.Label(conn_frame, text="Robot IP:").pack(side='left')
        self.ip_entry = ttk.Entry(conn_frame, width=15)
        self.ip_entry.insert(0, "192.168.4.1")
        self.ip_entry.pack(side='left', padx=5)
        
        ttk.Label(conn_frame, text="Port:").pack(side='left')
        self.port_entry = ttk.Entry(conn_frame, width=6)
        self.port_entry.insert(0, "8080")
        self.port_entry.pack(side='left', padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side='left', padx=10)
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.pack(side='left')
        
        # Data display
        notebook = ttk.Notebook(root)
        notebook.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Sensors tab
        sensors_frame = ttk.Frame(notebook)
        notebook.add(sensors_frame, text="Sensors")
        
        # IR Sensors
        ir_frame = ttk.LabelFrame(sensors_frame, text="IR Sensors (Edge Detection)", padding=10)
        ir_frame.pack(fill='x', padx=10, pady=5)
        
        self.ir_labels = []
        for i, name in enumerate(["A0 (Back-L)", "A1 (Front-L)", "A2 (Front-R)", "A3 (Back-R)"]):
            frame = ttk.Frame(ir_frame)
            frame.pack(fill='x', pady=2)
            ttk.Label(frame, text=name, width=15).pack(side='left')
            label = ttk.Label(frame, text="0.00V", width=10, font=('Courier', 12))
            label.pack(side='left', padx=10)
            self.ir_labels.append(label)
        
        # ToF Sensors
        tof_frame = ttk.LabelFrame(sensors_frame, text="ToF Sensors (Target Detection)", padding=10)
        tof_frame.pack(fill='x', padx=10, pady=5)
        
        self.tof_labels = []
        for i, name in enumerate(["R45°", "R23°", "Center", "L23°", "L45°"]):
            frame = ttk.Frame(tof_frame)
            frame.pack(fill='x', pady=2)
            ttk.Label(frame, text=name, width=15).pack(side='left')
            label = ttk.Label(frame, text="---mm", width=10, font=('Courier', 12))
            label.pack(side='left', padx=10)
            self.tof_labels.append(label)
        
        # State tab
        state_frame = ttk.Frame(notebook)
        notebook.add(state_frame, text="State")
        
        # Robot state
        state_info_frame = ttk.LabelFrame(state_frame, text="Robot State", padding=10)
        state_info_frame.pack(fill='x', padx=10, pady=5)
        
        self.state_label = ttk.Label(state_info_frame, text="IDLE", font=('Arial', 20, 'bold'))
        self.state_label.pack(pady=10)
        
        # Motors
        motor_frame = ttk.LabelFrame(state_frame, text="Motors", padding=10)
        motor_frame.pack(fill='x', padx=10, pady=5)
        
        frame = ttk.Frame(motor_frame)
        frame.pack(fill='x', pady=2)
        ttk.Label(frame, text="Left:", width=10).pack(side='left')
        self.motor_left_label = ttk.Label(frame, text="0.0", font=('Courier', 14))
        self.motor_left_label.pack(side='left', padx=10)
        
        frame = ttk.Frame(motor_frame)
        frame.pack(fill='x', pady=2)
        ttk.Label(frame, text="Right:", width=10).pack(side='left')
        self.motor_right_label = ttk.Label(frame, text="0.0", font=('Courier', 14))
        self.motor_right_label.pack(side='left', padx=10)
        
        # Console
        console_frame = ttk.LabelFrame(root, text="Console", padding=10)
        console_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.console = tk.Text(console_frame, height=8, state='disabled')
        self.console.pack(fill='both', expand=True)
        
        scrollbar = ttk.Scrollbar(self.console)
        scrollbar.pack(side='right', fill='y')
        self.console.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.console.yview)
        
        self.socket = None
        self.connected = False
    
    def log(self, message):
        self.console.config(state='normal')
        self.console.insert('end', f"{message}\n")
        self.console.see('end')
        self.console.config(state='disabled')
    
    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        try:
            ip = self.ip_entry.get()
            port = int(self.port_entry.get())
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(2.0)
            self.socket.connect((ip, port))
            
            self.connected = True
            self.status_label.config(text="Connected", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.log(f"Connected to {ip}:{port}")
            
            self.read_data()
        except Exception as e:
            self.log(f"Connection error: {e}")
            self.disconnect()
    
    def disconnect(self):
        if self.socket:
            self.socket.close()
            self.socket = None
        self.connected = False
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
        self.log("Disconnected")
    
    def read_data(self):
        if not self.connected:
            return
        
        try:
            data = self.socket.recv(4096).decode('utf-8')
            if not data:
                self.disconnect()
                return
            
            # Parse JSON lines
            for line in data.split('\n'):
                line = line.strip()
                if line:
                    try:
                        self.process_telemetry(json.loads(line))
                    except json.JSONDecodeError:
                        pass
            
            # Schedule next read
            self.root.after(50, self.read_data)
        except socket.timeout:
            self.root.after(50, self.read_data)
        except Exception as e:
            self.log(f"Read error: {e}")
            self.disconnect()
    
    def process_telemetry(self, data):
        # Update IR sensors
        if 'ir' in data:
            for i, voltage in enumerate(data['ir'][:4]):
                if i < len(self.ir_labels):
                    self.ir_labels[i].config(text=f"{voltage:.2f}V")
                    # Color code: red if > threshold
                    color = "red" if (i in [1,2] and voltage > 1.5) or (i in [0,3] and voltage > 3.0) else "black"
                    self.ir_labels[i].config(foreground=color)
        
        # Update ToF sensors
        if 'tof' in data:
            for i, distance in enumerate(data['tof'][:5]):
                if i < len(self.tof_labels):
                    if distance > 0:
                        self.tof_labels[i].config(text=f"{distance}mm")
                        # Color code: green if valid target range
                        color = "green" if 70 <= distance <= 1000 else "gray"
                        self.tof_labels[i].config(foreground=color)
                    else:
                        self.tof_labels[i].config(text="---mm", foreground="gray")
        
        # Update motors
        if 'm' in data and len(data['m']) >= 2:
            self.motor_left_label.config(text=f"{data['m'][0]:.1f}")
            self.motor_right_label.config(text=f"{data['m'][1]:.1f}")
        
        # Update state
        if 's' in data:
            states = ["IDLE", "CALIBRATING", "SEARCHING", "TRACKING", "ATTACKING", "EDGE_AVOIDING"]
            state_idx = data['s']
            if 0 <= state_idx < len(states):
                state_text = states[state_idx]
                self.state_label.config(text=state_text)
                # Color code states
                colors = {
                    "IDLE": "gray",
                    "CALIBRATING": "blue",
                    "SEARCHING": "orange",
                    "TRACKING": "yellow",
                    "ATTACKING": "red",
                    "EDGE_AVOIDING": "purple"
                }
                self.state_label.config(foreground=colors.get(state_text, "black"))

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotViewer(root)
    root.mainloop()
```

### Running the Simple Viewer

```bash
# Save the code above to a file
cd /Users/wici/Documents/GitHub/CTEA-BottleSumo/software/CAR/CAR_final

# Run it
python3 simple_viewer_v2.py
```

## Command Line Monitoring

### Using netcat (nc)
```bash
# View raw JSON stream
nc 192.168.4.1 8080

# Save to file for analysis
nc 192.168.4.1 8080 > telemetry_log.json
```

### Using Python Script
```python
# monitor.py
import socket
import json
import time

def monitor_robot(host='192.168.4.1', port=8080):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print(f"Connected to {host}:{port}")
    
    buffer = ""
    while True:
        try:
            data = sock.recv(1024).decode('utf-8')
            if not data:
                break
            
            buffer += data
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                if line.strip():
                    try:
                        telem = json.loads(line)
                        print(f"[{telem['t']:6d}] State:{telem['s']} "
                              f"IR:[{','.join(f'{v:.2f}' for v in telem['ir'])}] "
                              f"ToF:[{','.join(str(d) for d in telem['tof'])}] "
                              f"Motors:[{telem['m'][0]:.0f},{telem['m'][1]:.0f}]")
                    except json.JSONDecodeError:
                        pass
        except KeyboardInterrupt:
            break
    
    sock.close()

if __name__ == "__main__":
    monitor_robot()
```

Run with:
```bash
python3 monitor.py
```

## Troubleshooting

### Can't Connect to WiFi AP
```bash
# 1. Check robot is powered on
# 2. Wait 30 seconds after power on
# 3. Look for "BottleSumo_AP" in WiFi list
# 4. Check serial monitor for:
#    "[CORE1] WiFi AP: 192.168.4.1"

# 5. If still not visible, restart robot
```

### Connected but No Data
```bash
# Test with netcat
nc -v 192.168.4.1 8080

# If connection refused:
# - Check Core 1 initialized: "[CORE1] TCP Server started"
# - Check WiFi client connected: "[WiFi] Client connected"

# If connects but no data:
# - Robot may be in IDLE state (press button to start)
# - Check Core 0 is running: "[CORE0] Ready"
```

### Slow/Choppy Updates
```bash
# WiFi signal strength issue
# - Move closer to robot
# - Check for WiFi interference

# Or telemetry rate issue
# - Current rate: 2Hz (every 500ms)
# - This is normal for the time-sliced architecture
```

### Wrong Data Values
```bash
# IR sensors show 0.00V or 4.10V constantly
# - Check ADS1115 initialization: "[CORE1] ADS1115 OK"
# - Check I2C connections (SDA=2, SCL=3)

# ToF sensors show 0mm constantly
# - Check ToF initialization: "[CORE1] ToF sensors online: 5/5"
# - If less than 5, check XSHUT pins and I2C connections
```

## Advanced: Data Logging

### Log to CSV
```python
# log_telemetry.py
import socket
import json
import csv
import time
from datetime import datetime

def log_to_csv(host='192.168.4.1', port=8080):
    filename = f"telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'state', 'ir0', 'ir1', 'ir2', 'ir3',
                        'tof0', 'tof1', 'tof2', 'tof3', 'tof4',
                        'motor_left', 'motor_right'])
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        print(f"Logging to {filename}...")
        
        buffer = ""
        try:
            while True:
                data = sock.recv(1024).decode('utf-8')
                if not data:
                    break
                
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            telem = json.loads(line)
                            writer.writerow([
                                telem['t'], telem['s'],
                                *telem['ir'],
                                *telem['tof'],
                                *telem['m']
                            ])
                            f.flush()  # Write immediately
                        except (json.JSONDecodeError, KeyError):
                            pass
        except KeyboardInterrupt:
            print("\nLogging stopped")
        
        sock.close()

if __name__ == "__main__":
    log_to_csv()
```

## WiFi Network Details

### Access Point Configuration
- **SSID**: BottleSumo_AP
- **Password**: sumobot123456
- **IP**: 192.168.4.1
- **Subnet**: 192.168.4.0/24
- **Channel**: Auto (2.4GHz)
- **Max Clients**: 4

### Security
- **Encryption**: WPA2-PSK
- **No internet access** (robot is AP only)
- **Local network only** - robot does not route to internet

### Performance
- **WiFi Standard**: 802.11n (2.4GHz)
- **Bandwidth**: ~20 Mbps typical
- **Latency**: 10-50ms typical
- **Telemetry Rate**: 2Hz (500ms between updates)
- **Packet Size**: ~120 bytes per JSON message

## Summary

1. **Connect**: Join "BottleSumo_AP" WiFi (password: sumobot123456)
2. **Test**: `nc 192.168.4.1 8080`
3. **Visualize**: Use `simple_viewer_v2.py` for real-time GUI
4. **Log**: Use Python scripts for data analysis

The telemetry system provides continuous monitoring of all robot sensors and state, perfect for debugging, tuning, and competition analysis!
