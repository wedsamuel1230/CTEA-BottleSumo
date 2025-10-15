import socket
import json
import tkinter as tk
from tkinter import ttk
import threading
import time

class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("BottleSumo Motor Control")
        self.root.geometry("400x500")
        
        # Configuration - default values
        self.HOST = '192.168.4.1'
        self.PORT = 8080
        self.sock = None
        self.connected = False
        
        # Speed variable for commands
        self.command_speed = tk.IntVar(value=50)
        
        self.create_widgets()
        # Don't auto-connect, let user click connect button
    
    def create_widgets(self):
        # Connection controls
        connection_frame = ttk.LabelFrame(self.root, text="Connection", padding=8)
        connection_frame.pack(fill=tk.X, padx=10, pady=10)
        connection_frame.columnconfigure(1, weight=1)
        
        ttk.Label(connection_frame, text="Host:").grid(row=0, column=0, sticky="w")
        self.host_var = tk.StringVar(value=self.HOST)
        ttk.Entry(connection_frame, textvariable=self.host_var).grid(row=0, column=1, sticky="ew", padx=4)
        
        ttk.Label(connection_frame, text="Port:").grid(row=0, column=2, sticky="w")
        self.port_var = tk.IntVar(value=self.PORT)
        ttk.Entry(connection_frame, textvariable=self.port_var, width=8).grid(row=0, column=3, sticky="ew", padx=4)
        
        self.connect_button = ttk.Button(connection_frame, text="Connect", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=4, padx=(8, 0))
        
        # Connection status
        self.status_label = ttk.Label(self.root, text="Disconnected", foreground="red")
        self.status_label.pack(pady=10)
        
        # Ping button
        ttk.Button(self.root, text="Ping Server", command=self.ping_server).pack(pady=5)
        
        # Speed input for commands
        speed_frame = ttk.Frame(self.root)
        speed_frame.pack(pady=5)
        ttk.Label(speed_frame, text="Command Speed:").pack(side=tk.LEFT)
        ttk.Entry(speed_frame, textvariable=self.command_speed, width=5).pack(side=tk.LEFT)
        
        # Command buttons
        command_frame = ttk.LabelFrame(self.root, text="Commands")
        command_frame.pack(pady=10, padx=10, fill=tk.X)
        
        buttons = [
            ("Forward", "forward"),
            ("Backward", "backward"),
            ("Stop", "stop"),
            ("Rotate Left", "rotate_left"),
            ("Rotate Right", "rotate_right"),
            ("Revolve Left", "revolve_left"),
            ("Revolve Right", "revolve_right")
        ]
        
        for i, (text, cmd) in enumerate(buttons):
            row = i // 2
            col = i % 2
            ttk.Button(command_frame, text=text, command=lambda c=cmd: self.send_command(c)).grid(row=row, column=col, padx=5, pady=5, sticky=tk.W+tk.E)
        
        # Motor control sliders
        motor_frame = ttk.LabelFrame(self.root, text="Direct Motor Control")
        motor_frame.pack(pady=10, padx=10, fill=tk.X)
        
        # Motor 1 slider
        ttk.Label(motor_frame, text="Motor 1 (Left):").grid(row=0, column=0, sticky=tk.W)
        self.motor1_slider = tk.Scale(motor_frame, from_=-255, to=255, orient=tk.HORIZONTAL, command=self.on_motor1_change)
        self.motor1_slider.grid(row=0, column=1, sticky=tk.W+tk.E)
        self.motor1_label = ttk.Label(motor_frame, text="0")
        self.motor1_label.grid(row=0, column=2)
        
        # Motor 2 slider
        ttk.Label(motor_frame, text="Motor 2 (Right):").grid(row=1, column=0, sticky=tk.W)
        self.motor2_slider = tk.Scale(motor_frame, from_=-255, to=255, orient=tk.HORIZONTAL, command=self.on_motor2_change)
        self.motor2_slider.grid(row=1, column=1, sticky=tk.W+tk.E)
        self.motor2_label = ttk.Label(motor_frame, text="0")
        self.motor2_label.grid(row=1, column=2)
        
        # Stop motors button
        ttk.Button(motor_frame, text="Stop Motors", command=self.stop_motors).grid(row=2, column=0, columnspan=3, pady=10)
        
        # Response display
        response_frame = ttk.LabelFrame(self.root, text="Last Response")
        response_frame.pack(pady=10, padx=10, fill=tk.X)
        self.response_text = tk.Text(response_frame, height=3, wrap=tk.WORD)
        self.response_text.pack(fill=tk.X)
    
    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()
    
    def _connect(self):
        self.HOST = self.host_var.get()
        self.PORT = self.port_var.get()
        try:
            if self.sock:
                self.sock.close()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.HOST, self.PORT))
            self.connected = True
            self.status_label.config(text="Connected", foreground="green")
            self.connect_button.config(text="Disconnect")
            print("Connected to server")
        except Exception as e:
            self.status_label.config(text="Connection Failed", foreground="red")
            print(f"Connection failed: {e}")
            self.connected = False
    
    def _disconnect(self):
        if self.sock:
            self.sock.close()
        self.sock = None
        self.connected = False
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_button.config(text="Connect")
        print("Disconnected from server")
    
    def ping_server(self):
        if not self.connected:
            self.response_text.delete(1.0, tk.END)
            self.response_text.insert(tk.END, "Not connected to server")
            return
        
        data = {
            "command": "ping",
            "speed": 0
        }
        message = json.dumps(data) + '\n'
        
        try:
            self.sock.sendall(message.encode())
            response = self.sock.recv(1024)
            self.response_text.delete(1.0, tk.END)
            self.response_text.insert(tk.END, f"Ping response: {response.decode().strip()}")
        except Exception as e:
            self.response_text.delete(1.0, tk.END)
            self.response_text.insert(tk.END, f"Ping failed: {e}")
            self.connected = False
            self.status_label.config(text="Disconnected", foreground="red")
    
    def send_command(self, command):
        if not self.connected:
            self.response_text.delete(1.0, tk.END)
            self.response_text.insert(tk.END, "Not connected to server")
            return
        
        data = {
            "command": command,
            "speed": self.command_speed.get()
        }
        message = json.dumps(data) + '\n'
        
        try:
            self.sock.sendall(message.encode())
            response = self.sock.recv(1024)
            self.response_text.delete(1.0, tk.END)
            self.response_text.insert(tk.END, response.decode().strip())
        except Exception as e:
            self.response_text.delete(1.0, tk.END)
            self.response_text.insert(tk.END, f"Error: {e}")
    
    def send_direct_control(self, motor1_speed, motor2_speed):
        if not self.connected:
            self.response_text.delete(1.0, tk.END)
            self.response_text.insert(tk.END, "Not connected to server")
            return
        
        data = {
            "motor1": motor1_speed,
            "motor2": motor2_speed
        }
        message = json.dumps(data) + '\n'
        
        try:
            self.sock.sendall(message.encode())
            response = self.sock.recv(1024)
            self.response_text.delete(1.0, tk.END)
            self.response_text.insert(tk.END, response.decode().strip())
        except Exception as e:
            self.response_text.delete(1.0, tk.END)
            self.response_text.insert(tk.END, f"Error: {e}")
    
    def on_motor1_change(self, value):
        self.motor1_label.config(text=value)
        motor1 = int(float(value))
        motor2 = self.motor2_slider.get()
        self.send_direct_control(motor1, motor2)
    
    def on_motor2_change(self, value):
        self.motor2_label.config(text=value)
        motor1 = self.motor1_slider.get()
        motor2 = int(float(value))
        self.send_direct_control(motor1, motor2)
    
    def stop_motors(self):
        self.motor1_slider.set(0)
        self.motor2_slider.set(0)
        self.send_direct_control(0, 0)

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.mainloop()