#!/usr/bin/env python3
"""
Simple Viewer for BottleSumo CAR_final_v2
Connects to robot WiFi and displays real-time telemetry

Usage:
    python3 simple_viewer_v2.py

Connection:
    1. Connect to WiFi: BottleSumo_AP (password: sumobot123456)
    2. Run this script
    3. Click "Connect" button
"""

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
        
        # Timing info
        timing_frame = ttk.LabelFrame(state_frame, text="Timing", padding=10)
        timing_frame.pack(fill='x', padx=10, pady=5)
        
        frame = ttk.Frame(timing_frame)
        frame.pack(fill='x', pady=2)
        ttk.Label(frame, text="Uptime:", width=10).pack(side='left')
        self.uptime_label = ttk.Label(frame, text="0s", font=('Courier', 12))
        self.uptime_label.pack(side='left', padx=10)
        
        frame = ttk.Frame(timing_frame)
        frame.pack(fill='x', pady=2)
        ttk.Label(frame, text="Update Rate:", width=10).pack(side='left')
        self.rate_label = ttk.Label(frame, text="0 Hz", font=('Courier', 12))
        self.rate_label.pack(side='left', padx=10)
        
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
        self.last_timestamp = 0
        self.update_count = 0
        self.rate_start_time = 0
    
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
            
            import time
            self.rate_start_time = time.time()
            self.update_count = 0
            
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
        # Update counter for rate calculation
        self.update_count += 1
        
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
        
        # Update timing info
        if 't' in data:
            timestamp = data['t']
            self.uptime_label.config(text=f"{timestamp/1000:.1f}s")
            
            # Calculate update rate every 5 seconds
            import time
            elapsed = time.time() - self.rate_start_time
            if elapsed >= 5.0:
                rate = self.update_count / elapsed
                self.rate_label.config(text=f"{rate:.1f} Hz")
                self.rate_start_time = time.time()
                self.update_count = 0

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotViewer(root)
    root.mainloop()
