#!/usr/bin/env python3
"""
Pico W Car Tracker - Real-Time Telemetry Viewer

This GUI application connects to the Pico W car tracker via TCP and visualizes
real-time telemetry data including distance, direction bias, motor speeds, and
individual sensor readings.

Connection: 192.168.4.1:8080 (default)
Update Rate: 10Hz

Requirements:
    - Python 3.7+
    - tkinter (built-in)
    - matplotlib (pip install matplotlib)

Author: Autonomous Copilot Agent
Date: 2025-11-10
"""

import tkinter as tk
from tkinter import ttk, messagebox
import socket
import json
import threading
import time
from collections import deque
from datetime import datetime
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt


class TelemetryViewer:
    """Main GUI application for car tracker telemetry visualization"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Pico W Car Tracker - Telemetry Viewer")
        self.root.geometry("1200x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Connection settings
        self.host = tk.StringVar(value="192.168.4.1")
        self.port = tk.IntVar(value=8080)
        self.connected = False
        self.tcp_socket = None
        self.tcp_thread = None
        self.running = False
        
        # Telemetry data
        self.latest_data = {}
        self.data_lock = threading.Lock()
        
        # History for plotting (keep last 100 samples)
        self.history_size = 100
        self.time_history = deque(maxlen=self.history_size)
        self.dist_history = deque(maxlen=self.history_size)
        self.bias_history = deque(maxlen=self.history_size)
        self.speed_history = deque(maxlen=self.history_size)
        self.left_history = deque(maxlen=self.history_size)
        self.right_history = deque(maxlen=self.history_size)
        
        # Build UI
        self.create_widgets()
        
        # Start GUI update timer
        self.update_gui()
    
    def create_widgets(self):
        """Create all GUI widgets"""
        
        # ========== Top Control Panel ==========
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E))
        
        # Connection controls
        ttk.Label(control_frame, text="Host:").grid(row=0, column=0, padx=5)
        ttk.Entry(control_frame, textvariable=self.host, width=15).grid(row=0, column=1, padx=5)
        
        ttk.Label(control_frame, text="Port:").grid(row=0, column=2, padx=5)
        ttk.Entry(control_frame, textvariable=self.port, width=8).grid(row=0, column=3, padx=5)
        
        self.connect_btn = ttk.Button(control_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=10)
        
        self.status_label = ttk.Label(control_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=5, padx=10)
        
        # ========== Left Panel: Current Values ==========
        left_frame = ttk.LabelFrame(self.root, text="Current Telemetry", padding="10")
        left_frame.grid(row=1, column=0, sticky=(tk.N, tk.S, tk.W, tk.E), padx=10, pady=10)
        
        # Distance
        ttk.Label(left_frame, text="Distance:", font=("Arial", 12, "bold")).grid(row=0, column=0, sticky=tk.W, pady=5)
        self.dist_value = ttk.Label(left_frame, text="---", font=("Arial", 24))
        self.dist_value.grid(row=0, column=1, sticky=tk.W, pady=5)
        ttk.Label(left_frame, text="mm").grid(row=0, column=2, sticky=tk.W, pady=5)
        
        # Direction Bias
        ttk.Label(left_frame, text="Direction Bias:", font=("Arial", 12, "bold")).grid(row=1, column=0, sticky=tk.W, pady=5)
        self.bias_value = ttk.Label(left_frame, text="---", font=("Arial", 18))
        self.bias_value.grid(row=1, column=1, sticky=tk.W, pady=5)
        self.bias_indicator = ttk.Label(left_frame, text="", font=("Arial", 14))
        self.bias_indicator.grid(row=1, column=2, sticky=tk.W, pady=5)
        
        # Speed
        ttk.Label(left_frame, text="Speed:", font=("Arial", 12, "bold")).grid(row=2, column=0, sticky=tk.W, pady=5)
        self.speed_value = ttk.Label(left_frame, text="---", font=("Arial", 18))
        self.speed_value.grid(row=2, column=1, columnspan=2, sticky=tk.W, pady=5)
        
        # Motor speeds
        ttk.Label(left_frame, text="Left Motor:", font=("Arial", 12, "bold")).grid(row=3, column=0, sticky=tk.W, pady=5)
        self.left_value = ttk.Label(left_frame, text="---", font=("Arial", 16))
        self.left_value.grid(row=3, column=1, columnspan=2, sticky=tk.W, pady=5)
        
        ttk.Label(left_frame, text="Right Motor:", font=("Arial", 12, "bold")).grid(row=4, column=0, sticky=tk.W, pady=5)
        self.right_value = ttk.Label(left_frame, text="---", font=("Arial", 16))
        self.right_value.grid(row=4, column=1, columnspan=2, sticky=tk.W, pady=5)
        
        # Separator
        ttk.Separator(left_frame, orient=tk.HORIZONTAL).grid(row=5, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        
        # Sensor Array
        ttk.Label(left_frame, text="Sensor Array (mm):", font=("Arial", 12, "bold")).grid(row=6, column=0, columnspan=3, sticky=tk.W, pady=5)
        
        sensor_names = ["R45", "R23", "M0", "L23", "L45"]
        self.sensor_labels = {}
        for i, name in enumerate(sensor_names):
            ttk.Label(left_frame, text=f"{name}:").grid(row=7+i, column=0, sticky=tk.W, pady=2)
            label = ttk.Label(left_frame, text="---", font=("Arial", 14))
            label.grid(row=7+i, column=1, columnspan=2, sticky=tk.W, pady=2)
            self.sensor_labels[name] = label
        
        # Timestamp
        ttk.Separator(left_frame, orient=tk.HORIZONTAL).grid(row=12, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        ttk.Label(left_frame, text="Timestamp:", font=("Arial", 10)).grid(row=13, column=0, sticky=tk.W)
        self.timestamp_label = ttk.Label(left_frame, text="---", font=("Arial", 10))
        self.timestamp_label.grid(row=13, column=1, columnspan=2, sticky=tk.W)
        
        # ========== Right Panel: Plots ==========
        right_frame = ttk.LabelFrame(self.root, text="Real-Time Plots", padding="10")
        right_frame.grid(row=1, column=1, sticky=(tk.N, tk.S, tk.W, tk.E), padx=10, pady=10)
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(8, 10), dpi=100)
        
        # Distance plot
        self.ax_dist = self.fig.add_subplot(411)
        self.ax_dist.set_title("Distance (mm)")
        self.ax_dist.set_ylim(0, 1000)
        self.ax_dist.grid(True, alpha=0.3)
        self.line_dist, = self.ax_dist.plot([], [], 'b-', linewidth=2)
        
        # Direction Bias plot
        self.ax_bias = self.fig.add_subplot(412)
        self.ax_bias.set_title("Direction Bias (-1=Right, 0=Center, +1=Left)")
        self.ax_bias.set_ylim(-1.2, 1.2)
        self.ax_bias.axhline(0, color='gray', linestyle='--', alpha=0.5)
        self.ax_bias.grid(True, alpha=0.3)
        self.line_bias, = self.ax_bias.plot([], [], 'g-', linewidth=2)
        
        # Speed plot
        self.ax_speed = self.fig.add_subplot(413)
        self.ax_speed.set_title("Speed")
        self.ax_speed.set_ylim(-30, 100)
        self.ax_speed.axhline(0, color='gray', linestyle='--', alpha=0.5)
        self.ax_speed.grid(True, alpha=0.3)
        self.line_speed, = self.ax_speed.plot([], [], 'r-', linewidth=2)
        
        # Motor speeds plot
        self.ax_motors = self.fig.add_subplot(414)
        self.ax_motors.set_title("Motor Speeds (Left=Blue, Right=Red)")
        self.ax_motors.set_ylim(-30, 100)
        self.ax_motors.set_xlabel("Time (samples)")
        self.ax_motors.axhline(0, color='gray', linestyle='--', alpha=0.5)
        self.ax_motors.grid(True, alpha=0.3)
        self.line_left, = self.ax_motors.plot([], [], 'b-', linewidth=2, label="Left")
        self.line_right, = self.ax_motors.plot([], [], 'r-', linewidth=2, label="Right")
        self.ax_motors.legend()
        
        self.fig.tight_layout()
        
        # Embed matplotlib in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Configure grid weights for resizing
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(1, weight=1)
    
    def toggle_connection(self):
        """Connect or disconnect from TCP server"""
        if not self.connected:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        """Connect to Pico W TCP server"""
        try:
            host = self.host.get()
            port = self.port.get()
            
            # Create TCP socket
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.settimeout(5.0)  # 5 second connection timeout
            self.tcp_socket.connect((host, port))
            self.tcp_socket.settimeout(None)  # Non-blocking after connection
            
            self.connected = True
            self.running = True
            
            # Start TCP receive thread
            self.tcp_thread = threading.Thread(target=self.tcp_receive_loop, daemon=True)
            self.tcp_thread.start()
            
            # Update UI
            self.status_label.config(text="Connected", foreground="green")
            self.connect_btn.config(text="Disconnect")
            
            print(f"[INFO] Connected to {host}:{port}")
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")
            self.disconnect()
    
    def disconnect(self):
        """Disconnect from TCP server"""
        self.running = False
        self.connected = False
        
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except:
                pass
            self.tcp_socket = None
        
        # Update UI
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
        
        print("[INFO] Disconnected")
    
    def tcp_receive_loop(self):
        """Background thread to receive TCP data"""
        buffer = ""
        
        try:
            while self.running:
                # Receive data
                data = self.tcp_socket.recv(4096).decode('utf-8')
                if not data:
                    print("[WARNING] Connection closed by server")
                    break
                
                buffer += data
                
                # Process complete JSON lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.parse_telemetry(line)
        
        except Exception as e:
            print(f"[ERROR] TCP receive error: {e}")
        
        finally:
            # Auto-disconnect on error
            self.root.after(0, self.disconnect)
    
    def parse_telemetry(self, json_str):
        """Parse JSON telemetry string and update data"""
        try:
            data = json.loads(json_str)
            
            with self.data_lock:
                self.latest_data = data
                
                # Add to history
                current_time = time.time()
                self.time_history.append(current_time)
                self.dist_history.append(data.get('dist', 0))
                self.bias_history.append(data.get('bias', 0.0))
                self.speed_history.append(data.get('speed', 0.0))
                self.left_history.append(data.get('left', 0.0))
                self.right_history.append(data.get('right', 0.0))
        
        except json.JSONDecodeError as e:
            print(f"[ERROR] JSON parse error: {e}")
    
    def update_gui(self):
        """Update GUI with latest telemetry data (called periodically)"""
        
        with self.data_lock:
            if self.latest_data:
                data = self.latest_data.copy()
                
                # Update text values
                self.dist_value.config(text=f"{data.get('dist', 0)}")
                
                bias = data.get('bias', 0.0)
                self.bias_value.config(text=f"{bias:+.2f}")
                
                # Direction indicator
                if bias > 0.1:
                    self.bias_indicator.config(text="← LEFT", foreground="blue")
                elif bias < -0.1:
                    self.bias_indicator.config(text="RIGHT →", foreground="orange")
                else:
                    self.bias_indicator.config(text="CENTER", foreground="green")
                
                self.speed_value.config(text=f"{data.get('speed', 0.0):.1f}")
                self.left_value.config(text=f"{data.get('left', 0.0):.1f}")
                self.right_value.config(text=f"{data.get('right', 0.0):.1f}")
                
                # Update sensor readings
                sensor_keys = ['R45', 'R23', 'M0', 'L23', 'L45']
                for key in sensor_keys:
                    value = data.get(key, 0)
                    text = f"{value} mm" if value > 0 else "----"
                    self.sensor_labels[key].config(text=text)
                
                # Timestamp
                ts = data.get('timestamp', 0)
                self.timestamp_label.config(text=f"{ts} ms")
                
                # Update plots
                if len(self.time_history) > 1:
                    # Create relative time axis (last N seconds)
                    times = list(self.time_history)
                    x_axis = [(t - times[0]) for t in times]
                    
                    # Distance plot
                    self.line_dist.set_data(x_axis, list(self.dist_history))
                    self.ax_dist.set_xlim(0, max(x_axis) if x_axis else 10)
                    
                    # Bias plot
                    self.line_bias.set_data(x_axis, list(self.bias_history))
                    self.ax_bias.set_xlim(0, max(x_axis) if x_axis else 10)
                    
                    # Speed plot
                    self.line_speed.set_data(x_axis, list(self.speed_history))
                    self.ax_speed.set_xlim(0, max(x_axis) if x_axis else 10)
                    
                    # Motor plots
                    self.line_left.set_data(x_axis, list(self.left_history))
                    self.line_right.set_data(x_axis, list(self.right_history))
                    self.ax_motors.set_xlim(0, max(x_axis) if x_axis else 10)
                    
                    # Redraw canvas
                    self.canvas.draw_idle()
        
        # Schedule next update (100ms = 10Hz)
        self.root.after(100, self.update_gui)
    
    def on_closing(self):
        """Handle window close event"""
        self.disconnect()
        self.root.destroy()


def main():
    """Main entry point"""
    root = tk.Tk()
    app = TelemetryViewer(root)
    root.mainloop()


if __name__ == "__main__":
    main()
