"""Bottle Sumo Real-Time TCP Viewer - COMPACT Edition with TEST_MODE Protocol.

This script connects to the Bottle Sumo robot's TCP streaming server (port 4242),
parses the JSON telemetry stream, and visualizes the data in a COMPACT Tkinter GUI.

COMPACT DESIGN: Uses tabbed interface to fit 768px screens (650px window height).
TEST_MODE PROTOCOL: Integrated motor control and mode switching commands.

Features:
- Tabbed interface (Sensors | Motors | Config | Status)
- TEST_MODE protocol support (SET_MODE, TEST_MOTOR, STOP_MOTOR)
- Per-sensor IR threshold configuration
- Color-coded sensor visualization
- ToF proximity indicators
- Real-time telemetry streaming

Usage:
    python viewer_compact.py
"""
from __future__ import annotations

import json
import queue
import socket
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import messagebox, ttk
from typing import Any, Dict, Optional

# Default connection settings (matches firmware TCP_SERVER_PORT = 4242)
DEFAULT_HOST = "192.168.42.1"
DEFAULT_PORT = 4242

# Socket settings
SOCKET_RECV_BYTES = 4096
SOCKET_TIMEOUT = 1.0  # seconds


@dataclass
class TelemetryPacket:
    """Container for a parsed telemetry JSON document."""

    timestamp: int = 0
    sensors_raw: list[int] = field(default_factory=list)
    sensors_voltage: list[float] = field(default_factory=list)
    tof_distances: list[int] = field(default_factory=list)
    tof_valid: list[bool] = field(default_factory=list)
    tof_status: list[int] = field(default_factory=list)
    tof_object_direction: str = ""
    robot_state: Dict[str, Any] = field(default_factory=dict)
    system_info: Dict[str, Any] = field(default_factory=dict)
    ir_edge_thresholds: list[float] = field(default_factory=lambda: [2.5, 2.5, 2.5, 2.5])
    tof_detection_threshold: int = 1600

    @classmethod
    def from_json(cls, payload: Dict[str, Any]) -> "TelemetryPacket":
        """Create a packet from the decoded JSON dictionary."""
        # IR sensors (new: 'irsensors', legacy: 'sensors')
        sensors = payload.get("irsensors", payload.get("sensors", {}))
        
        # ToF sensors (new: 'tof', legacy: 'tofsensors')
        tofsensors = payload.get("tof", payload.get("tofsensors", {}))
        
        distances = tofsensors.get("distance_mm", tofsensors.get("distances", []))
        direction = tofsensors.get("direction", tofsensors.get("object_direction", ""))
        
        # Handle edge_threshold - can be list (new) or single value (legacy)
        edge_threshold_data = sensors.get("edge_threshold", [2.5, 2.5, 2.5, 2.5])
        if isinstance(edge_threshold_data, list):
            ir_edge_thresholds = [float(t) for t in edge_threshold_data]
        else:
            ir_edge_thresholds = [float(edge_threshold_data)] * 4

        return cls(
            timestamp=payload.get("timestamp", 0),
            sensors_raw=list(sensors.get("raw", [])),
            sensors_voltage=list(sensors.get("voltage", [])),
            tof_distances=list(distances),
            tof_valid=list(tofsensors.get("valid", [])),
            tof_status=list(tofsensors.get("status", [])),
            tof_object_direction=direction,
            robot_state=dict(payload.get("robot_state", {})),
            system_info=dict(payload.get("system_info", {})),
            ir_edge_thresholds=ir_edge_thresholds,
            tof_detection_threshold=int(tofsensors.get("detection_threshold", 1600)),
        )


class TelemetryReader(threading.Thread):
    """Background thread that maintains the TCP connection and feeds packets into a queue."""

    def __init__(
        self,
        host: str,
        port: int,
        output_queue: "queue.Queue[TelemetryPacket | Exception]",
    ) -> None:
        super().__init__(name="TelemetryReader", daemon=True)
        self._host = host
        self._port = port
        self._output_queue = output_queue
        self._sock: Optional[socket.socket] = None
        self._stop_event = threading.Event()
        self._buffer = ""

    def stop(self) -> None:
        self._stop_event.set()
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            finally:
                try:
                    self._sock.close()
                except OSError:
                    pass

    def run(self) -> None:
        try:
            self._sock = socket.create_connection((self._host, self._port), timeout=SOCKET_TIMEOUT)
            self._sock.settimeout(SOCKET_TIMEOUT)
        except OSError as exc:
            self._output_queue.put(exc)
            return

        while not self._stop_event.is_set():
            try:
                chunk = self._sock.recv(SOCKET_RECV_BYTES)
            except socket.timeout:
                continue
            except OSError as exc:
                self._output_queue.put(exc)
                break

            if not chunk:
                self._output_queue.put(ConnectionError("Connection closed by remote host."))
                break

            self._buffer += chunk.decode("utf-8", errors="ignore")
            *complete_messages, self._buffer = self._buffer.split("\n")

            for message in complete_messages:
                message = message.strip()
                if not message:
                    continue
                try:
                    payload = json.loads(message)
                    
                    # Check if this is an acknowledgment/error message (not telemetry)
                    if "ack" in payload or "error" in payload:
                        self._output_queue.put({"_type": "ack", "payload": payload})
                    else:
                        packet = TelemetryPacket.from_json(payload)
                        self._output_queue.put(packet)
                except json.JSONDecodeError as exc:
                    self._output_queue.put(exc)

        self.stop()


class BottleSumoViewerCompact(tk.Tk):
    """COMPACT Tkinter GUI for Bottle Sumo telemetry with TEST_MODE protocol integration."""

    def __init__(self) -> None:
        super().__init__()
        self.title("Bottle Sumo Viewer - COMPACT")
        self.geometry("700x650")  # COMPACT: Fits 768px screens
        self.minsize(650, 600)

        self._reader_thread: Optional[TelemetryReader] = None
        self._queue: "queue.Queue[TelemetryPacket | Exception]" = queue.Queue()

        # TEST_MODE protocol variables
        self.motor_left_var = tk.IntVar(value=0)  # -100 to +100
        self.motor_right_var = tk.IntVar(value=0)  # -100 to +100
        self.test_mode_var = tk.StringVar(value="AUTO")  # Current test mode
        
        # IR threshold config (per-sensor)
        self.ir_threshold_configs = [tk.DoubleVar(value=2.5) for _ in range(4)]
        
        # Last packet for threshold marker redraws
        self._last_packet: Optional[TelemetryPacket] = None
        self.initial_threshold_displayed = False
        
        # Sensor colors (per-sensor threshold bars)
        self.sensor_colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4']

        self._setup_progress_bar_styles()
        self._build_ui()
        self._schedule_queue_processing()

    def _setup_progress_bar_styles(self) -> None:
        """Configure ttk.Style for color-coded progress bars."""
        style = ttk.Style()
        
        # IR Sensor styles
        style.configure('IR.green.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#4CAF50')
        style.configure('IR.yellow.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#FFC107')
        style.configure('IR.red.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#F44336')
        
        # ToF Sensor styles
        style.configure('ToF.green.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#4CAF50')
        style.configure('ToF.yellow.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#FFC107')
        style.configure('ToF.red.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#F44336')

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=6)
        root.grid(row=0, column=0, sticky="nsew")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        # Connection controls (compact, single row)
        conn_frame = ttk.Frame(root)
        conn_frame.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        conn_frame.columnconfigure(1, weight=1)

        ttk.Label(conn_frame, text="Host:").grid(row=0, column=0, sticky="w", padx=(0,2))
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(conn_frame, textvariable=self.host_var, width=15).grid(row=0, column=1, sticky="ew", padx=2)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=2, sticky="w", padx=(4,2))
        self.port_var = tk.IntVar(value=DEFAULT_PORT)
        ttk.Entry(conn_frame, textvariable=self.port_var, width=6).grid(row=0, column=3, padx=2)

        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=4, padx=(4, 0))
        
        # COMPACT: Tabbed interface (major space saver)
        notebook = ttk.Notebook(root)
        notebook.grid(row=1, column=0, sticky="nsew", pady=(4, 0))
        root.rowconfigure(1, weight=1)
        
        # TAB 1: IR Sensors + Thresholds
        self._build_sensors_tab(notebook)
        
        # TAB 2: Motors + Mode Control
        self._build_motors_tab(notebook)
        
        # TAB 3: ToF Sensors
        self._build_tof_tab(notebook)
        
        # TAB 4: Robot State + System Info
        self._build_status_tab(notebook)

        # Status bar (compact, always visible)
        self.status_var = tk.StringVar(value="Idle")
        status_bar = ttk.Label(root, textvariable=self.status_var, relief=tk.SUNKEN, anchor="w")
        status_bar.grid(row=2, column=0, sticky="ew", pady=(4, 0))

    def _build_sensors_tab(self, notebook: ttk.Notebook) -> None:
        """TAB 1: IR Sensors with per-sensor thresholds."""
        tab = ttk.Frame(notebook, padding=6)
        notebook.add(tab, text="IR Sensors")
        
        # Sensor readings table
        ttk.Label(tab, text="IR Sensor", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=0, sticky="w", padx=2, pady=2)
        ttk.Label(tab, text="Raw", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=1, sticky="w", padx=2, pady=2)
        ttk.Label(tab, text="Voltage", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=2, sticky="w", padx=2, pady=2)
        ttk.Label(tab, text="Level", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=3, sticky="ew", padx=2, pady=2)
        tab.columnconfigure(3, weight=1)
        
        self.sensor_raw_labels: list[tk.StringVar] = []
        self.sensor_voltage_labels: list[tk.StringVar] = []
        self.sensor_voltage_bars: list[tk.Canvas] = []
        
        for idx in range(4):
            ttk.Label(tab, text=f"Sensor {idx}").grid(row=idx+1, column=0, sticky="w", padx=2, pady=1)
            
            raw_var = tk.StringVar(value="-")
            ttk.Label(tab, textvariable=raw_var, width=6).grid(row=idx+1, column=1, sticky="w", padx=2)
            
            volt_var = tk.StringVar(value="-")
            ttk.Label(tab, textvariable=volt_var, width=8).grid(row=idx+1, column=2, sticky="w", padx=2)
            
            # Canvas for voltage bar + threshold marker
            canvas = tk.Canvas(tab, width=200, height=18, highlightthickness=0, bg='#e0e0e0')
            canvas.grid(row=idx+1, column=3, sticky="ew", padx=2, pady=1)
            
            self.sensor_raw_labels.append(raw_var)
            self.sensor_voltage_labels.append(volt_var)
            self.sensor_voltage_bars.append(canvas)
        
        # Threshold configuration (compact, 2 columns)
        ttk.Separator(tab, orient='horizontal').grid(row=5, column=0, columnspan=4, sticky="ew", pady=4)
        ttk.Label(tab, text="Threshold Config", font=('TkDefaultFont', 9, 'bold')).grid(row=6, column=0, columnspan=4, sticky="w", padx=2)
        
        self.threshold_spinboxes = []
        for idx in range(4):
            row_idx = 7 + idx
            col_offset = 0 if idx < 2 else 2
            local_row = 7 + (idx % 2)
            
            ttk.Label(tab, text=f"S{idx}:").grid(row=local_row, column=col_offset, sticky="w", padx=2)
            spinbox = ttk.Spinbox(tab, from_=0.0, to=4.096, increment=0.1,
                                  textvariable=self.ir_threshold_configs[idx], width=6)
            spinbox.grid(row=local_row, column=col_offset+1, sticky="w", padx=2)
            self.threshold_spinboxes.append(spinbox)
        
        ttk.Button(tab, text="Apply All", command=self._apply_all_thresholds).grid(row=9, column=0, columnspan=2, sticky="ew", padx=2, pady=4)

    def _build_motors_tab(self, notebook: ttk.Notebook) -> None:
        """TAB 2: Motors + Mode Control (TEST_MODE protocol)."""
        tab = ttk.Frame(notebook, padding=6)
        notebook.add(tab, text="Motors")
        
        # Mode selection
        ttk.Label(tab, text="Test Mode:", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=0, sticky="w", padx=2, pady=2)
        mode_combo = ttk.Combobox(tab, textvariable=self.test_mode_var, 
                                  values=["AUTO", "TEST_MOTOR", "TEST_SENSOR", "CALIBRATE_IR", "CALIBRATE_TOF"],
                                  state="readonly", width=15)
        mode_combo.grid(row=0, column=1, sticky="w", padx=2, pady=2)
        mode_combo.bind("<<ComboboxSelected>>", lambda e: self._send_set_mode_command())
        
        ttk.Separator(tab, orient='horizontal').grid(row=1, column=0, columnspan=3, sticky="ew", pady=4)
        
        # Left motor
        ttk.Label(tab, text="Left Motor PWM:", font=('TkDefaultFont', 9, 'bold')).grid(row=2, column=0, sticky="w", padx=2, pady=2)
        self.motor_left_scale = tk.Scale(tab, from_=-100, to=100, orient=tk.HORIZONTAL, 
                                         variable=self.motor_left_var, length=250, resolution=5,
                                         command=self._on_motor_change, showvalue=0)
        self.motor_left_scale.grid(row=2, column=1, sticky="ew", padx=2)
        ttk.Label(tab, textvariable=self.motor_left_var, width=5).grid(row=2, column=2, sticky="w", padx=2)
        tab.columnconfigure(1, weight=1)
        
        # Right motor
        ttk.Label(tab, text="Right Motor PWM:", font=('TkDefaultFont', 9, 'bold')).grid(row=3, column=0, sticky="w", padx=2, pady=2)
        self.motor_right_scale = tk.Scale(tab, from_=-100, to=100, orient=tk.HORIZONTAL,
                                          variable=self.motor_right_var, length=250, resolution=5,
                                          command=self._on_motor_change, showvalue=0)
        self.motor_right_scale.grid(row=3, column=1, sticky="ew", padx=2)
        ttk.Label(tab, textvariable=self.motor_right_var, width=5).grid(row=3, column=2, sticky="w", padx=2)
        
        # Control buttons (compact row)
        btn_frame = ttk.Frame(tab)
        btn_frame.grid(row=4, column=0, columnspan=3, sticky="ew", pady=6)
        btn_frame.columnconfigure(0, weight=1)
        btn_frame.columnconfigure(1, weight=1)
        
        ttk.Button(btn_frame, text="⚡ Send Motor Command", command=self._send_test_motor_command).grid(row=0, column=0, sticky="ew", padx=2)
        ttk.Button(btn_frame, text="🛑 STOP Motors", command=self._send_stop_motor_command).grid(row=0, column=1, sticky="ew", padx=2)
        
        # Status display
        ttk.Separator(tab, orient='horizontal').grid(row=5, column=0, columnspan=3, sticky="ew", pady=4)
        ttk.Label(tab, text="Motor Status:", font=('TkDefaultFont', 9, 'bold')).grid(row=6, column=0, sticky="w", padx=2)
        self.motor_status_var = tk.StringVar(value="Idle")
        ttk.Label(tab, textvariable=self.motor_status_var, foreground="blue").grid(row=6, column=1, columnspan=2, sticky="w", padx=2)

    def _build_tof_tab(self, notebook: ttk.Notebook) -> None:
        """TAB 3: ToF Sensors."""
        tab = ttk.Frame(notebook, padding=6)
        notebook.add(tab, text="ToF Sensors")
        
        ttk.Label(tab, text="Direction", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=0, sticky="w", padx=2, pady=2)
        ttk.Label(tab, text="Distance (mm)", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=1, sticky="w", padx=2, pady=2)
        ttk.Label(tab, text="Status", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=2, sticky="w", padx=2, pady=2)
        ttk.Label(tab, text="Proximity", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=3, sticky="ew", padx=2, pady=2)
        tab.columnconfigure(3, weight=1)
        
        self.tof_distance_labels: list[tk.StringVar] = []
        self.tof_status_labels: list[tk.StringVar] = []
        self.tof_proximity_bars: list[ttk.Progressbar] = []
        tof_directions = ["Right", "Front", "Left"]
        
        for idx, direction in enumerate(tof_directions):
            ttk.Label(tab, text=direction).grid(row=idx+1, column=0, sticky="w", padx=2, pady=1)
            
            distance_var = tk.StringVar(value="-")
            ttk.Label(tab, textvariable=distance_var, width=10).grid(row=idx+1, column=1, sticky="w", padx=2)
            
            status_var = tk.StringVar(value="-")
            ttk.Label(tab, textvariable=status_var, width=12).grid(row=idx+1, column=2, sticky="w", padx=2)
            
            proximity_bar = ttk.Progressbar(tab, mode='determinate', length=200, maximum=1500)
            proximity_bar.grid(row=idx+1, column=3, sticky="ew", padx=2, pady=1)
            
            self.tof_distance_labels.append(distance_var)
            self.tof_status_labels.append(status_var)
            self.tof_proximity_bars.append(proximity_bar)
        
        ttk.Separator(tab, orient='horizontal').grid(row=4, column=0, columnspan=4, sticky="ew", pady=4)
        ttk.Label(tab, text="Object Direction:").grid(row=5, column=0, sticky="w", padx=2)
        self.tof_object_direction_var = tk.StringVar(value="-")
        ttk.Label(tab, textvariable=self.tof_object_direction_var, font=('TkDefaultFont', 9, 'bold')).grid(row=5, column=1, columnspan=3, sticky="w", padx=2)

    def _build_status_tab(self, notebook: ttk.Notebook) -> None:
        """TAB 4: Robot State + System Info."""
        tab = ttk.Frame(notebook, padding=6)
        notebook.add(tab, text="Status")
        
        # Robot state section
        ttk.Label(tab, text="Robot State", font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=0, columnspan=2, sticky="w", padx=2, pady=2)
        
        self.action_var = tk.StringVar(value="-")
        ttk.Label(tab, text="Action:").grid(row=1, column=0, sticky="w", padx=2)
        ttk.Label(tab, textvariable=self.action_var).grid(row=1, column=1, sticky="w", padx=2)
        
        self.edge_detected_var = tk.StringVar(value="-")
        ttk.Label(tab, text="Edge Detected:").grid(row=2, column=0, sticky="w", padx=2)
        ttk.Label(tab, textvariable=self.edge_detected_var).grid(row=2, column=1, sticky="w", padx=2)
        
        self.danger_level_var = tk.StringVar(value="-")
        ttk.Label(tab, text="Danger Level:").grid(row=3, column=0, sticky="w", padx=2)
        ttk.Label(tab, textvariable=self.danger_level_var).grid(row=3, column=1, sticky="w", padx=2)
        
        ttk.Separator(tab, orient='horizontal').grid(row=4, column=0, columnspan=2, sticky="ew", pady=4)
        
        # System info section
        ttk.Label(tab, text="System Info", font=('TkDefaultFont', 9, 'bold')).grid(row=5, column=0, columnspan=2, sticky="w", padx=2, pady=2)
        
        self.timestamp_var = tk.StringVar(value="-")
        ttk.Label(tab, text="Timestamp (ms):").grid(row=6, column=0, sticky="w", padx=2)
        ttk.Label(tab, textvariable=self.timestamp_var).grid(row=6, column=1, sticky="w", padx=2)
        
        self.core0_freq_var = tk.StringVar(value="-")
        ttk.Label(tab, text="Core0 freq (Hz):").grid(row=7, column=0, sticky="w", padx=2)
        ttk.Label(tab, textvariable=self.core0_freq_var).grid(row=7, column=1, sticky="w", padx=2)
        
        self.core1_freq_var = tk.StringVar(value="-")
        ttk.Label(tab, text="Core1 freq (Hz):").grid(row=8, column=0, sticky="w", padx=2)
        ttk.Label(tab, textvariable=self.core1_freq_var).grid(row=8, column=1, sticky="w", padx=2)
        
        self.wifi_rssi_var = tk.StringVar(value="-")
        ttk.Label(tab, text="WiFi RSSI (dBm):").grid(row=9, column=0, sticky="w", padx=2)
        ttk.Label(tab, textvariable=self.wifi_rssi_var).grid(row=9, column=1, sticky="w", padx=2)
        
        self.free_heap_var = tk.StringVar(value="-")
        ttk.Label(tab, text="Free heap (bytes):").grid(row=10, column=0, sticky="w", padx=2)
        ttk.Label(tab, textvariable=self.free_heap_var).grid(row=10, column=1, sticky="w", padx=2)

    # ==================================================================
    # Connection handling
    # ==================================================================
    def _toggle_connection(self) -> None:
        if self._reader_thread and self._reader_thread.is_alive():
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        host = self.host_var.get().strip() or DEFAULT_HOST
        try:
            port = int(self.port_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid Port", "Please enter a valid integer port number.")
            return

        self.status_var.set(f"Connecting to {host}:{port}…")
        self.connect_button.config(text="Disconnect")

        self._reader_thread = TelemetryReader(host, port, self._queue)
        self._reader_thread.start()

    def _disconnect(self) -> None:
        if self._reader_thread:
            self.status_var.set("Disconnecting…")
            self._reader_thread.stop()
            self._reader_thread.join(timeout=2.0)
            self._reader_thread = None
        self.connect_button.config(text="Connect")
        self.status_var.set("Disconnected")

    # ==================================================================
    # TEST_MODE Protocol Commands
    # ==================================================================
    def _send_set_mode_command(self) -> None:
        """Send SET_MODE <mode> command to firmware."""
        if not self._reader_thread or not self._reader_thread.is_alive():
            self.status_var.set("⚠️ Not connected")
            return
        
        mode = self.test_mode_var.get()
        command = f"SET_MODE {mode}\n"
        self._send_tcp_command(command)
        self.status_var.set(f"📤 Sent: SET_MODE {mode}")

    def _send_test_motor_command(self) -> None:
        """Send TEST_MOTOR <left> <right> command to firmware."""
        if not self._reader_thread or not self._reader_thread.is_alive():
            self.status_var.set("⚠️ Not connected")
            return
        
        left = self.motor_left_var.get()
        right = self.motor_right_var.get()
        command = f"TEST_MOTOR {left} {right}\n"
        self._send_tcp_command(command)
        self.motor_status_var.set(f"📤 Sent: L={left}, R={right}")

    def _send_stop_motor_command(self) -> None:
        """Send STOP_MOTOR command to firmware."""
        if not self._reader_thread or not self._reader_thread.is_alive():
            self.status_var.set("⚠️ Not connected")
            return
        
        command = "STOP_MOTOR\n"
        self._send_tcp_command(command)
        self.motor_status_var.set("🛑 STOP sent")
        self.motor_left_var.set(0)
        self.motor_right_var.set(0)

    def _on_motor_change(self, value=None) -> None:
        """Called when motor sliders change (no auto-send, user must click button)."""
        pass  # Explicit button press required for TEST_MODE protocol

    def _apply_all_thresholds(self) -> None:
        """Apply all IR thresholds via TCP commands."""
        if not self._reader_thread or not self._reader_thread.is_alive():
            self.status_var.set("⚠️ Not connected")
            return
        
        for idx in range(4):
            threshold = round(self.ir_threshold_configs[idx].get(), 2)
            command = json.dumps({"cmd":"set_threshold","sensor":idx,"value":threshold}) + "\n"
            self._send_tcp_command(command)
        
        self.status_var.set("📤 Sent threshold commands (awaiting acks...)")

    def _send_tcp_command(self, command: str) -> None:
        """Send raw TCP command to robot."""
        try:
            if hasattr(self._reader_thread, '_sock') and self._reader_thread._sock:
                self._reader_thread._sock.sendall(command.encode('utf-8'))
                print(f"[TCP] Sent: {command.strip()}")
            else:
                self.status_var.set("❌ Socket not available")
        except Exception as exc:
            self.status_var.set(f"❌ Send error: {exc}")
            print(f"[TCP] Error: {exc}")

    # ==================================================================
    # Queue processing and display updates
    # ==================================================================
    def _schedule_queue_processing(self) -> None:
        self.after(100, self._process_queue)

    def _process_queue(self) -> None:
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                break

            if isinstance(item, Exception):
                self.status_var.set(str(item))
                if isinstance(item, json.JSONDecodeError):
                    continue
                messagebox.showerror("Connection Error", str(item))
                self._disconnect()
                break
            
            # Check if this is an acknowledgment message
            if isinstance(item, dict) and item.get("_type") == "ack":
                self._handle_acknowledgment(item["payload"])
                continue

            assert isinstance(item, TelemetryPacket)
            self._update_display(item)

        self._schedule_queue_processing()

    def _handle_acknowledgment(self, payload: dict) -> None:
        """Handle acknowledgment or error response from firmware."""
        if "ack" in payload:
            if payload["ack"] == "set_threshold":
                sensor = payload.get("sensor", 0)
                if payload.get("status") == "ok":
                    value = payload.get("value", 0)
                    self.status_var.set(f"✅ Sensor {sensor} threshold: {value:.2f}V")
                else:
                    error = payload.get("error", "unknown")
                    self.status_var.set(f"❌ Sensor {sensor} error: {error}")
            elif payload["ack"] == "test_motor":
                left = payload.get("left_pwm", 0)
                right = payload.get("right_pwm", 0)
                timeout = payload.get("timeout_s", 0)
                self.motor_status_var.set(f"✅ ACK: L={left:.0f}, R={right:.0f}, Timeout={timeout}s")
            elif payload["ack"] == "stop_motor":
                self.motor_status_var.set("✅ Motors stopped")
            elif payload["ack"] == "set_mode":
                mode = payload.get("mode", "")
                self.status_var.set(f"✅ Mode switched to: {mode}")
        elif "error" in payload:
            error_msg = payload.get("error", "unknown")
            self.status_var.set(f"❌ Error: {error_msg}")

    def _update_display(self, packet: TelemetryPacket) -> None:
        """Update GUI with new telemetry packet."""
        self._last_packet = packet
        self.status_var.set(time.strftime("Last update: %H:%M:%S"))
        
        # IR sensors
        for idx in range(4):
            raw = packet.sensors_raw[idx] if idx < len(packet.sensors_raw) else "-"
            volt = packet.sensors_voltage[idx] if idx < len(packet.sensors_voltage) else "-"
            
            self.sensor_raw_labels[idx].set(str(raw))
            self.sensor_voltage_labels[idx].set(f"{volt:.3f}" if isinstance(volt, (float, int)) else "-")
            
            # Draw voltage bar on canvas
            if isinstance(volt, (float, int)):
                self._draw_voltage_bar(idx, float(volt))
            else:
                self._draw_voltage_bar(idx, 0.0)
        
        # ToF sensors
        for idx in range(3):
            dist = packet.tof_distances[idx] if idx < len(packet.tof_distances) else None
            valid = packet.tof_valid[idx] if idx < len(packet.tof_valid) else False
            status = packet.tof_status[idx] if idx < len(packet.tof_status) else None
            
            if isinstance(dist, int) and valid:
                self.tof_distance_labels[idx].set(f"{dist} mm")
                self.tof_status_labels[idx].set("✓ Valid")
                
                # Proximity bar (inverse: closer = more filled)
                proximity = max(0, 1500 - min(1500, dist))
                self.tof_proximity_bars[idx]['value'] = proximity
                
                # Color coding
                if dist >= 1000:
                    self.tof_proximity_bars[idx].configure(style='ToF.green.Horizontal.TProgressbar')
                elif dist >= 350:
                    self.tof_proximity_bars[idx].configure(style='ToF.yellow.Horizontal.TProgressbar')
                else:
                    self.tof_proximity_bars[idx].configure(style='ToF.red.Horizontal.TProgressbar')
            else:
                self.tof_distance_labels[idx].set("-")
                self.tof_status_labels[idx].set(f"Error ({status})" if status is not None else "-")
                self.tof_proximity_bars[idx]['value'] = 0
        
        self.tof_object_direction_var.set(packet.tof_object_direction or "-")
        
        # Robot state
        self.action_var.set(packet.robot_state.get("action", "-"))
        self.edge_detected_var.set(str(packet.robot_state.get("edge_detected", "-")))
        self.danger_level_var.set(str(packet.robot_state.get("danger_level", "-")))
        
        # System info
        self.timestamp_var.set(str(packet.timestamp))
        self.core0_freq_var.set(str(packet.system_info.get("core0_freq", "-")))
        self.core1_freq_var.set(str(packet.system_info.get("core1_freq", "-")))
        self.wifi_rssi_var.set(str(packet.system_info.get("wifi_rssi", "-")))
        self.free_heap_var.set(str(packet.system_info.get("free_heap", "-")))

    def _draw_voltage_bar(self, sensor_idx: int, voltage: float) -> None:
        """Draw voltage bar on Canvas with threshold marker."""
        canvas = self.sensor_voltage_bars[sensor_idx]
        canvas.delete('all')
        
        width = canvas.winfo_width() if canvas.winfo_width() > 1 else 200
        height = canvas.winfo_height() if canvas.winfo_height() > 1 else 18
        
        # Voltage bar (0-4.096V range)
        fill_width = (voltage / 4.096) * width
        fill_color = self.sensor_colors[sensor_idx]
        
        if fill_width > 0:
            canvas.create_rectangle(0, 0, fill_width, height, fill=fill_color, outline='')
        
        # Threshold marker
        threshold = self.ir_threshold_configs[sensor_idx].get()
        line_x = (threshold / 4.096) * width
        canvas.create_line(line_x, 0, line_x, height, fill='white', width=3, tags='outline')
        canvas.create_line(line_x, 0, line_x, height, fill='black', width=1, tags='threshold')

    def destroy(self) -> None:
        """Cleanup on window close."""
        self._disconnect()
        super().destroy()


def main() -> None:
    """Application entry point."""
    app = BottleSumoViewerCompact()
    app.mainloop()


if __name__ == "__main__":
    main()
