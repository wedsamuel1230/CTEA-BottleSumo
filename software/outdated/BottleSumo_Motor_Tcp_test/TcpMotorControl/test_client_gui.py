#!/usr/bin/env python3
"""
TCP Motor Control GUI Test Client

Graphical interface for testing and controlling the BottleSumo robot.
Provides real-time control, status monitoring, and connection management.

Usage:
    python test_client_gui.py
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import socket
import json
import threading
import time
from datetime import datetime

class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("BottleSumo TCP Motor Control")
        self.root.geometry("900x700")
        self.root.resizable(True, True)
        
        # Connection state
        self.sock = None
        self.connected = False
        self.authenticated = False
        self.status_thread = None
        self.status_running = False
        
        # Create UI
        self.create_widgets()
        
    def create_widgets(self):
        # ==================== Connection Frame ====================
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N), padx=10, pady=5)
        
        ttk.Label(conn_frame, text="Host:").grid(row=0, column=0, sticky=tk.W, padx=5)
        self.host_entry = ttk.Entry(conn_frame, width=20)
        self.host_entry.insert(0, "192.168.42.1")
        self.host_entry.grid(row=0, column=1, padx=5)
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=2, sticky=tk.W, padx=5)
        self.port_entry = ttk.Entry(conn_frame, width=10)
        self.port_entry.insert(0, "5000")
        self.port_entry.grid(row=0, column=3, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="● Disconnected", foreground="red")
        self.status_label.grid(row=0, column=5, padx=10)
        
        # ==================== Authentication Frame ====================
        auth_frame = ttk.LabelFrame(self.root, text="Authentication", padding=10)
        auth_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N), padx=10, pady=5)
        
        ttk.Label(auth_frame, text="Token:").grid(row=0, column=0, sticky=tk.W, padx=5)
        self.token_entry = ttk.Entry(auth_frame, width=30, show="*")
        self.token_entry.insert(0, "BottleSumo2025Secure")
        self.token_entry.grid(row=0, column=1, padx=5)
        
        self.auth_btn = ttk.Button(auth_frame, text="Authenticate", command=self.authenticate, state=tk.DISABLED)
        self.auth_btn.grid(row=0, column=2, padx=5)
        
        self.auth_status = ttk.Label(auth_frame, text="Not authenticated", foreground="orange")
        self.auth_status.grid(row=0, column=3, padx=10)
        
        # ==================== Motor Control Frame ====================
        motor_frame = ttk.LabelFrame(self.root, text="Motor Control", padding=10)
        motor_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=10, pady=5)
        
        # Left Motor
        ttk.Label(motor_frame, text="Left Motor:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky=tk.W, pady=5)
        self.left_scale = tk.Scale(motor_frame, from_=255, to=-255, orient=tk.VERTICAL, 
                                   length=200, width=30, command=self.on_slider_change)
        self.left_scale.set(0)
        self.left_scale.grid(row=1, column=0, padx=20)
        self.left_value_label = ttk.Label(motor_frame, text="0", font=("Arial", 12))
        self.left_value_label.grid(row=2, column=0)
        
        # Control Buttons
        btn_frame = ttk.Frame(motor_frame)
        btn_frame.grid(row=1, column=1, padx=20)
        
        self.forward_btn = ttk.Button(btn_frame, text="⬆ Forward", width=15, command=lambda: self.quick_move(150, 150))
        self.forward_btn.grid(row=0, column=1, pady=5)
        
        self.left_btn = ttk.Button(btn_frame, text="⬅ Turn Left", width=15, command=lambda: self.quick_move(-100, 100))
        self.left_btn.grid(row=1, column=0, padx=5)
        
        self.stop_btn = ttk.Button(btn_frame, text="■ STOP", width=15, command=self.stop_motors, 
                                   style="Stop.TButton")
        self.stop_btn.grid(row=1, column=1, pady=5)
        
        self.right_btn = ttk.Button(btn_frame, text="➡ Turn Right", width=15, command=lambda: self.quick_move(100, -100))
        self.right_btn.grid(row=1, column=2, padx=5)
        
        self.reverse_btn = ttk.Button(btn_frame, text="⬇ Reverse", width=15, command=lambda: self.quick_move(-150, -150))
        self.reverse_btn.grid(row=2, column=1, pady=5)
        
        ttk.Separator(btn_frame, orient=tk.HORIZONTAL).grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        
        self.estop_btn = ttk.Button(btn_frame, text="⚠ EMERGENCY STOP", width=20, command=self.emergency_stop,
                                    style="Emergency.TButton")
        self.estop_btn.grid(row=4, column=0, columnspan=3, pady=10)
        
        ttk.Button(btn_frame, text="Apply Sliders", width=20, command=self.apply_sliders).grid(row=5, column=0, columnspan=3, pady=5)
        
        # Right Motor
        ttk.Label(motor_frame, text="Right Motor:", font=("Arial", 10, "bold")).grid(row=0, column=2, sticky=tk.W, pady=5)
        self.right_scale = tk.Scale(motor_frame, from_=255, to=-255, orient=tk.VERTICAL,
                                    length=200, width=30, command=self.on_slider_change)
        self.right_scale.set(0)
        self.right_scale.grid(row=1, column=2, padx=20)
        self.right_value_label = ttk.Label(motor_frame, text="0", font=("Arial", 12))
        self.right_value_label.grid(row=2, column=2)
        
        # Disable motor controls initially
        self.set_motor_controls_state(tk.DISABLED)
        
        # ==================== Status Frame ====================
        status_frame = ttk.LabelFrame(self.root, text="Robot Status", padding=10)
        status_frame.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=10, pady=5)
        
        self.status_text = scrolledtext.ScrolledText(status_frame, width=35, height=15, state=tk.DISABLED)
        self.status_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        ttk.Button(status_frame, text="Refresh Status", command=self.get_status).grid(row=1, column=0, pady=5)
        
        self.auto_refresh_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(status_frame, text="Auto-refresh (1 Hz)", variable=self.auto_refresh_var,
                       command=self.toggle_auto_refresh).grid(row=2, column=0)
        
        # ==================== Sensor Calibration Frame ====================
        calib_frame = ttk.LabelFrame(self.root, text="Sensor Calibration", padding=10)
        calib_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), padx=10, pady=5)
        
        ttk.Label(calib_frame, text="Samples:").grid(row=0, column=0, padx=5)
        self.samples_entry = ttk.Entry(calib_frame, width=10)
        self.samples_entry.insert(0, "64")
        self.samples_entry.grid(row=0, column=1, padx=5)
        
        ttk.Button(calib_frame, text="Calibrate (Auto)", command=self.calibrate_sensors).grid(row=0, column=2, padx=5)
        
        # ==================== Log Frame ====================
        log_frame = ttk.LabelFrame(self.root, text="Command Log", padding=10)
        log_frame.grid(row=3, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, width=35, height=8, state=tk.DISABLED)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        ttk.Button(log_frame, text="Clear Log", command=self.clear_log).grid(row=1, column=0, pady=5)
        
        # Configure grid weights for resizing
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(2, weight=1)
        
        # Style configuration
        style = ttk.Style()
        style.configure("Stop.TButton", foreground="red", font=("Arial", 10, "bold"))
        style.configure("Emergency.TButton", foreground="red", font=("Arial", 11, "bold"))
        
    def log(self, message, level="INFO"):
        """Add message to log with timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {level}: {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        
    def clear_log(self):
        """Clear the command log"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
        
    def update_status_display(self, text):
        """Update status text area"""
        self.status_text.config(state=tk.NORMAL)
        self.status_text.delete(1.0, tk.END)
        self.status_text.insert(1.0, text)
        self.status_text.config(state=tk.DISABLED)
        
    def toggle_connection(self):
        """Toggle connection state"""
        if self.connected:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        """Establish TCP connection"""
        host = self.host_entry.get()
        port = int(self.port_entry.get())
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(15.0)  # Increased timeout for slower responses
            self.sock.connect((host, port))
            self.connected = True
            
            self.status_label.config(text="● Connected", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.auth_btn.config(state=tk.NORMAL)
            self.log(f"Connected to {host}:{port}", "SUCCESS")
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")
            self.log(f"Connection failed: {e}", "ERROR")
            
    def disconnect(self):
        """Close TCP connection"""
        if self.sock:
            self.sock.close()
            self.sock = None
            
        self.connected = False
        self.authenticated = False
        self.status_running = False
        
        self.status_label.config(text="● Disconnected", foreground="red")
        self.auth_status.config(text="Not authenticated", foreground="orange")
        self.connect_btn.config(text="Connect")
        self.auth_btn.config(state=tk.DISABLED)
        self.set_motor_controls_state(tk.DISABLED)
        self.log("Disconnected", "INFO")
        
    def send_command(self, cmd_dict):
        """Send JSON command and receive response"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to robot first")
            return None
            
        try:
            cmd_json = json.dumps(cmd_dict)
            self.sock.sendall((cmd_json + '\n').encode('utf-8'))
            self.log(f"→ {cmd_json}", "SEND")
            
            # Receive response with improved buffering
            response = b''
            start_time = time.time()
            while True:
                # Check for timeout
                if time.time() - start_time > 10.0:
                    raise socket.timeout("Response timeout after 10 seconds")
                
                # Try to receive data in larger chunks
                try:
                    chunk = self.sock.recv(1024)
                    if not chunk:
                        break
                    response += chunk
                    
                    # Check if we have a complete line (ending with \n)
                    if b'\n' in response:
                        # Extract first complete line
                        response = response.split(b'\n')[0]
                        break
                except socket.timeout:
                    # If we have partial data, continue waiting
                    if response:
                        continue
                    else:
                        raise
                
            resp_json = json.loads(response.decode('utf-8'))
            self.log(f"← {json.dumps(resp_json)}", "RECV")
            return resp_json
            
        except Exception as e:
            self.log(f"Command error: {e}", "ERROR")
            messagebox.showerror("Command Error", f"Failed to send command: {e}")
            self.disconnect()
            return None
            
    def authenticate(self):
        """Authenticate with robot"""
        token = self.token_entry.get()
        resp = self.send_command({"action": "auth", "token": token})
        
        if resp and resp.get("status") == "ok":
            self.authenticated = True
            self.auth_status.config(text="✓ Authenticated", foreground="green")
            self.set_motor_controls_state(tk.NORMAL)
            messagebox.showinfo("Success", "Authentication successful!")
        else:
            error_msg = resp.get("message", "Unknown error") if resp else "No response"
            messagebox.showerror("Authentication Failed", f"Failed to authenticate: {error_msg}")
            
    def set_motor_controls_state(self, state):
        """Enable/disable motor control buttons"""
        controls = [
            self.forward_btn, self.reverse_btn, self.left_btn, self.right_btn,
            self.stop_btn, self.estop_btn, self.left_scale, self.right_scale
        ]
        for control in controls:
            control.config(state=state)
            
    def on_slider_change(self, value):
        """Update slider value labels"""
        left_val = self.left_scale.get()
        right_val = self.right_scale.get()
        self.left_value_label.config(text=str(left_val))
        self.right_value_label.config(text=str(right_val))
        
    def apply_sliders(self):
        """Apply current slider values to motors"""
        if not self.authenticated:
            messagebox.showwarning("Not Authenticated", "Please authenticate first")
            return
            
        left = self.left_scale.get()
        right = self.right_scale.get()
        self.send_motor_command(left, right)
        
    def quick_move(self, left, right):
        """Quick movement button handler"""
        if not self.authenticated:
            messagebox.showwarning("Not Authenticated", "Please authenticate first")
            return
            
        self.send_motor_command(left, right)
        
    def send_motor_command(self, left, right):
        """Send set command to motors"""
        resp = self.send_command({"action": "set", "left": int(left), "right": int(right)})
        if not resp or resp.get("status") != "ok":
            error_msg = resp.get("message", "Unknown error") if resp else "No response"
            messagebox.showerror("Command Failed", f"Motor command failed: {error_msg}")
            
    def stop_motors(self):
        """Stop all motors"""
        if not self.authenticated:
            return
            
        resp = self.send_command({"action": "stop"})
        if resp and resp.get("status") == "ok":
            self.left_scale.set(0)
            self.right_scale.set(0)
            
    def emergency_stop(self):
        """Trigger emergency stop"""
        if not self.connected:
            return
            
        result = messagebox.askyesno("Emergency Stop", 
                                     "This will trigger emergency stop and latch the system.\n\n" +
                                     "Continue?",
                                     icon="warning")
        if result:
            resp = self.send_command({"action": "estop"})
            if resp and resp.get("status") == "ok":
                self.left_scale.set(0)
                self.right_scale.set(0)
                messagebox.showinfo("Emergency Stop", "Emergency stop activated!\n\n" +
                                   "System is now latched. Power cycle may be required.")
                
    def get_status(self):
        """Request and display robot status"""
        if not self.authenticated:
            messagebox.showwarning("Not Authenticated", "Please authenticate first")
            return
            
        resp = self.send_command({"action": "status"})
        if resp and resp.get("status") == "ok":
            # Format status display
            status_str = "=== ROBOT STATUS ===\n\n"
            status_str += f"Authentication: {'✓ Yes' if resp.get('auth') else '✗ No'}\n\n"
            
            motors = resp.get('motors', {})
            status_str += f"Motors:\n"
            status_str += f"  Left:  {motors.get('left', 0):4d}\n"
            status_str += f"  Right: {motors.get('right', 0):4d}\n\n"
            
            sensors = resp.get('sensors', {})
            raw = sensors.get('raw', [0, 0, 0, 0])
            flags = sensors.get('flags', [False, False, False, False])
            status_str += f"Sensors (Raw):\n"
            status_str += f"  FL: {raw[0]:5d} {'[EDGE]' if flags[0] else ''}\n"
            status_str += f"  FR: {raw[1]:5d} {'[EDGE]' if flags[1] else ''}\n"
            status_str += f"  RL: {raw[2]:5d} {'[EDGE]' if flags[2] else ''}\n"
            status_str += f"  RR: {raw[3]:5d} {'[EDGE]' if flags[3] else ''}\n\n"
            
            safety = resp.get('safety', {})
            status_str += f"Safety:\n"
            status_str += f"  E-Stop:  {'✓ ACTIVE' if safety.get('estop') else '✗ Inactive'}\n"
            status_str += f"  Latched: {'✓ YES' if safety.get('latched') else '✗ No'}\n"
            
            self.update_status_display(status_str)
            
    def toggle_auto_refresh(self):
        """Toggle automatic status refresh"""
        if self.auto_refresh_var.get():
            self.status_running = True
            self.status_thread = threading.Thread(target=self.status_updater, daemon=True)
            self.status_thread.start()
        else:
            self.status_running = False
            
    def status_updater(self):
        """Background thread for status updates"""
        while self.status_running and self.authenticated:
            self.get_status()
            time.sleep(1.0)
            
    def calibrate_sensors(self):
        """Calibrate edge sensors"""
        if not self.authenticated:
            messagebox.showwarning("Not Authenticated", "Please authenticate first")
            return
            
        samples = int(self.samples_entry.get())
        
        result = messagebox.showinfo("Calibration", 
                                    "Place robot on WHITE surface (center of ring).\n\n" +
                                    f"Will take {samples} samples.\n\n" +
                                    "Click OK to start calibration.",
                                    icon="info")
        
        resp = self.send_command({"action": "calibrate", "mode": "auto", "samples": samples})
        
        if resp and resp.get("status") == "ok":
            messagebox.showinfo("Success", "Calibration completed successfully!")
        else:
            error_msg = resp.get("message", "Unknown error") if resp else "No response"
            messagebox.showerror("Calibration Failed", f"Failed to calibrate: {error_msg}")
            
    def on_closing(self):
        """Handle window close event"""
        if self.connected:
            self.disconnect()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()
