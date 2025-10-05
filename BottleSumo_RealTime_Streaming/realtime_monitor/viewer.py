"""Bottle Sumo Real-Time TCP viewer.

This script connects to the Bottle Sumo robot's TCP streaming server (default port 4242),
parses the JSON telemetry stream, and visualises the data in a Tkinter GUI.

The robot firmware periodically sends newline-delimited JSON objects that contain
sensor readings, robot state, and system information. The GUI displays the data and
keeps updating as new packets arrive.
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

# Default connection settings that match the firmware (`TCP_SERVER_PORT = 4242`).
DEFAULT_HOST = "192.168.42.1"
DEFAULT_PORT = 4242

# Socket settings.
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
    ir_edge_threshold: float = 2.5  # Default IR edge detection threshold (volts)
    tof_detection_threshold: int = 1600  # Default ToF detection threshold (mm)

    @classmethod
    def from_json(cls, payload: Dict[str, Any]) -> "TelemetryPacket":
        """Create a packet from the decoded JSON dictionary."""
        # IR sensors (legacy name 'sensors' or new name 'irsensors')
        sensors = payload.get("irsensors", payload.get("sensors", {}))
        
        # ToF sensors - support both 'tof' (new firmware) and 'tofsensors' (legacy)
        tofsensors = payload.get("tof", payload.get("tofsensors", {}))
        
        # Map field names: firmware uses 'distance_mm' and 'direction', viewer expects 'distances' and 'object_direction'
        distances = tofsensors.get("distance_mm", tofsensors.get("distances", []))
        direction = tofsensors.get("direction", tofsensors.get("object_direction", ""))
        
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
            ir_edge_threshold=float(sensors.get("edge_threshold", 2.5)),
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

    def run(self) -> None:  # pragma: no cover - thread loop is hard to unit-test here
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
                continue  # allow stop checks
            except OSError as exc:
                self._output_queue.put(exc)
                break

            if not chunk:
                # Remote closed the connection
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
                    packet = TelemetryPacket.from_json(payload)
                    self._output_queue.put(packet)
                except json.JSONDecodeError as exc:
                    self._output_queue.put(exc)

        self.stop()


class BottleSumoViewer(tk.Tk):
    """Tkinter application that visualises telemetry updates."""

    def __init__(self) -> None:
        super().__init__()
        self.title("Bottle Sumo Real-Time Viewer")
        self.geometry("650x750")  # Increased height for threshold config UI
        self.minsize(600, 750)

        self._reader_thread: Optional[TelemetryReader] = None
        self._queue: "queue.Queue[TelemetryPacket | Exception]" = queue.Queue()

        # Variables for window dragging
        self._drag_start_x = 0
        self._drag_start_y = 0
        
        # Configurable threshold (user can adjust via UI)
        self.ir_threshold_config = tk.DoubleVar(value=2.5)  # Default 2.5V
        
        # Configure custom progress bar styles for color coding
        self._setup_progress_bar_styles()

        self._build_ui()
        self._setup_window_drag()
        self._schedule_queue_processing()

    # ------------------------------------------------------------------
    # UI construction helpers
    # ------------------------------------------------------------------
    def _setup_progress_bar_styles(self) -> None:
        """Configure ttk.Style for color-coded progress bars (green/yellow/red zones)."""
        style = ttk.Style()
        
        # IR Sensor styles (voltage-based)
        style.configure('IR.green.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#4CAF50')  # Green
        style.configure('IR.yellow.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#FFC107')  # Yellow
        style.configure('IR.red.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#F44336')  # Red
        
        # ToF Sensor styles (distance-based, inverse logic: closer = more danger)
        style.configure('ToF.green.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#4CAF50')  # Green
        style.configure('ToF.yellow.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#FFC107')  # Yellow
        style.configure('ToF.red.Horizontal.TProgressbar', troughcolor='#e0e0e0', background='#F44336')  # Red
    
    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=12)
        root.grid(row=0, column=0, sticky="nsew")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        # Connection controls -------------------------------------------------
        connection_frame = ttk.LabelFrame(root, text="Connection", padding=8)
        connection_frame.grid(row=0, column=0, sticky="ew")
        connection_frame.columnconfigure(1, weight=1)

        ttk.Label(connection_frame, text="Host:").grid(row=0, column=0, sticky="w")
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(connection_frame, textvariable=self.host_var).grid(row=0, column=1, sticky="ew", padx=4)

        ttk.Label(connection_frame, text="Port:").grid(row=0, column=2, sticky="w")
        self.port_var = tk.IntVar(value=DEFAULT_PORT)
        ttk.Entry(connection_frame, textvariable=self.port_var, width=8).grid(row=0, column=3, sticky="ew", padx=4)

        self.connect_button = ttk.Button(connection_frame, text="Connect", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=4, padx=(8, 0))

        # Sensor data ---------------------------------------------------------
        sensors_frame = ttk.LabelFrame(root, text="Sensors", padding=8)
        sensors_frame.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        sensors_frame.columnconfigure(1, weight=1)
        sensors_frame.columnconfigure(3, weight=1)

        ttk.Label(sensors_frame, text="Index").grid(row=0, column=0, padx=4, sticky="w")
        ttk.Label(sensors_frame, text="Raw").grid(row=0, column=1, padx=4, sticky="w")
        ttk.Label(sensors_frame, text="Voltage (V)").grid(row=0, column=2, padx=4, sticky="w")
        ttk.Label(sensors_frame, text="Level").grid(row=0, column=3, padx=4, sticky="w")

        self.sensor_raw_labels: list[tk.StringVar] = []
        self.sensor_voltage_labels: list[tk.StringVar] = []
        self.sensor_voltage_bars: list[ttk.Progressbar] = []
        self.sensor_threshold_markers: list[tk.Canvas] = []  # Canvas overlays for threshold lines
        
        for idx in range(4):
            ttk.Label(sensors_frame, text=f"Sensor {idx}").grid(row=idx + 1, column=0, sticky="w", padx=5)
            raw_var = tk.StringVar(value="-")
            volt_var = tk.StringVar(value="-")
            ttk.Label(sensors_frame, textvariable=raw_var).grid(row=idx + 1, column=1, sticky="w", padx=5)
            ttk.Label(sensors_frame, textvariable=volt_var).grid(row=idx + 1, column=2, sticky="w", padx=5)
            
            # Add progress bar for voltage visualization with Canvas overlay for threshold marker
            bar_container = tk.Frame(sensors_frame, relief=tk.SUNKEN, borderwidth=1)
            bar_container.grid(row=idx + 1, column=3, sticky="ew", padx=5)
            
            progress_bar = ttk.Progressbar(bar_container, mode='determinate', length=150, maximum=4.096, 
                                           style='IR.green.Horizontal.TProgressbar')
            progress_bar.pack(fill='both', expand=True)
            
            # Canvas overlay for red threshold line (2px wide, positioned dynamically)
            # Note: No bg parameter = transparent overlay over progress bar
            threshold_canvas = tk.Canvas(bar_container, width=150, height=20, highlightthickness=0)
            threshold_canvas.place(x=0, y=0, relwidth=1.0, relheight=1.0)
            
            # Bind mouse events for draggable threshold marker
            threshold_canvas.bind("<Enter>", self._on_threshold_marker_enter)
            threshold_canvas.bind("<Leave>", self._on_threshold_marker_leave)
            threshold_canvas.bind("<Button-1>", self._on_threshold_drag_start)
            threshold_canvas.bind("<B1-Motion>", self._on_threshold_drag_motion)
            threshold_canvas.bind("<ButtonRelease-1>", self._on_threshold_drag_end)
            
            self.sensor_raw_labels.append(raw_var)
            self.sensor_voltage_labels.append(volt_var)
            self.sensor_voltage_bars.append(progress_bar)
            self.sensor_threshold_markers.append(threshold_canvas)

        # Threshold Configuration ---------------------------------------------------
        threshold_frame = ttk.LabelFrame(root, text="Threshold Configuration", padding=8)
        threshold_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        threshold_frame.columnconfigure(1, weight=1)
        
        # IR Edge Threshold (user configurable)
        ttk.Label(threshold_frame, text="IR Edge Threshold (V):").grid(row=0, column=0, sticky="w", padx=5)
        threshold_spinbox = ttk.Spinbox(threshold_frame, from_=0.0, to=4.096, increment=0.1, 
                                        textvariable=self.ir_threshold_config, width=8)
        threshold_spinbox.grid(row=0, column=1, sticky="w", padx=5)
        ttk.Button(threshold_frame, text="Apply", command=self._apply_threshold_changes).grid(row=0, column=2, padx=5)
        
        # Display firmware's threshold values (read-only)
        ttk.Label(threshold_frame, text="Firmware IR Threshold:").grid(row=1, column=0, sticky="w", padx=5)
        self.firmware_ir_threshold_var = tk.StringVar(value="-")
        ttk.Label(threshold_frame, textvariable=self.firmware_ir_threshold_var).grid(row=1, column=1, sticky="w", padx=5)
        
        # ToF Sensors data ---------------------------------------------------
        tof_frame = ttk.LabelFrame(root, text="ToF Sensors (VL53L0X)", padding=8)
        tof_frame.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        tof_frame.columnconfigure(3, weight=1)  # Make proximity bar column expandable

        ttk.Label(tof_frame, text="Direction").grid(row=0, column=0, padx=4, sticky="w")
        ttk.Label(tof_frame, text="Distance (mm)").grid(row=0, column=1, padx=4, sticky="w")
        ttk.Label(tof_frame, text="Status").grid(row=0, column=2, padx=4, sticky="w")
        ttk.Label(tof_frame, text="Proximity").grid(row=0, column=3, padx=4, sticky="w")

        self.tof_distance_labels: list[tk.StringVar] = []
        self.tof_status_labels: list[tk.StringVar] = []
        self.tof_proximity_bars: list[ttk.Progressbar] = []
        tof_directions = ["Right", "Front", "Left"]
        
        for idx, direction in enumerate(tof_directions):
            ttk.Label(tof_frame, text=direction).grid(row=idx + 1, column=0, sticky="w", padx=5)
            distance_var = tk.StringVar(value="-")
            status_var = tk.StringVar(value="-")
            ttk.Label(tof_frame, textvariable=distance_var).grid(row=idx + 1, column=1, sticky="w", padx=5)
            ttk.Label(tof_frame, textvariable=status_var).grid(row=idx + 1, column=2, sticky="w", padx=5)
            
            # Add progress bar for proximity visualization (0-2000mm range, inverse display)
            # Bar fills MORE as object gets CLOSER (provides intuitive threat/proximity indication)
            proximity_bar = ttk.Progressbar(tof_frame, mode='determinate', length=150, maximum=2000)
            proximity_bar.grid(row=idx + 1, column=3, sticky="ew", padx=5)
            
            self.tof_distance_labels.append(distance_var)
            self.tof_status_labels.append(status_var)
            self.tof_proximity_bars.append(proximity_bar)

        self.tof_object_direction_var = tk.StringVar(value="-")
        ttk.Label(tof_frame, text="Object Direction:").grid(row=4, column=0, sticky="w")
        ttk.Label(tof_frame, textvariable=self.tof_object_direction_var).grid(row=4, column=1, columnspan=2, sticky="w")

        # Robot state --------------------------------------------------------
        state_frame = ttk.LabelFrame(root, text="Robot State", padding=8)
        state_frame.grid(row=4, column=0, sticky="ew", pady=(10, 0))
        state_frame.columnconfigure(1, weight=1)

        self.action_var = tk.StringVar(value="-")
        ttk.Label(state_frame, text="Action:").grid(row=0, column=0, sticky="w")
        ttk.Label(state_frame, textvariable=self.action_var).grid(row=0, column=1, sticky="w")

        self.edge_detected_var = tk.StringVar(value="-")
        ttk.Label(state_frame, text="Edge Detected:").grid(row=1, column=0, sticky="w")
        ttk.Label(state_frame, textvariable=self.edge_detected_var).grid(row=1, column=1, sticky="w")

        self.edge_direction_var = tk.StringVar(value="-")
        ttk.Label(state_frame, text="Edge Direction:").grid(row=2, column=0, sticky="w")
        ttk.Label(state_frame, textvariable=self.edge_direction_var).grid(row=2, column=1, sticky="w")

        self.danger_level_var = tk.StringVar(value="-")
        ttk.Label(state_frame, text="Danger Level:").grid(row=3, column=0, sticky="w")
        ttk.Label(state_frame, textvariable=self.danger_level_var).grid(row=3, column=1, sticky="w")

        # System info --------------------------------------------------------
        system_frame = ttk.LabelFrame(root, text="System Info", padding=8)
        system_frame.grid(row=5, column=0, sticky="ew", pady=(10, 0))
        system_frame.columnconfigure(1, weight=1)

        self.timestamp_var = tk.StringVar(value="-")
        ttk.Label(system_frame, text="Timestamp (ms):").grid(row=0, column=0, sticky="w")
        ttk.Label(system_frame, textvariable=self.timestamp_var).grid(row=0, column=1, sticky="w")

        self.core0_freq_var = tk.StringVar(value="-")
        ttk.Label(system_frame, text="Core0 freq (Hz):").grid(row=1, column=0, sticky="w")
        ttk.Label(system_frame, textvariable=self.core0_freq_var).grid(row=1, column=1, sticky="w")

        self.core1_freq_var = tk.StringVar(value="-")
        ttk.Label(system_frame, text="Core1 freq (Hz):").grid(row=2, column=0, sticky="w")
        ttk.Label(system_frame, textvariable=self.core1_freq_var).grid(row=2, column=1, sticky="w")

        self.wifi_rssi_var = tk.StringVar(value="-")
        ttk.Label(system_frame, text="WiFi RSSI (dBm):").grid(row=3, column=0, sticky="w")
        ttk.Label(system_frame, textvariable=self.wifi_rssi_var).grid(row=3, column=1, sticky="w")

        self.free_heap_var = tk.StringVar(value="-")
        ttk.Label(system_frame, text="Free heap (bytes):").grid(row=4, column=0, sticky="w")
        ttk.Label(system_frame, textvariable=self.free_heap_var).grid(row=4, column=1, sticky="w")

        # Status bar ---------------------------------------------------------
        self.status_var = tk.StringVar(value="Idle")
        status_bar = ttk.Label(root, textvariable=self.status_var, relief=tk.SUNKEN, anchor="w")
        status_bar.grid(row=6, column=0, sticky="ew", pady=(10, 0))

        root.rowconfigure(1, weight=1)
        
        # Initialize threshold markers after UI is built
        self._update_threshold_markers()

    def _apply_threshold_changes(self) -> None:
        """Apply threshold changes: update GUI markers and attempt to send command to firmware.
        
        Note: Firmware threshold is currently a compile-time constant (constexpr) and cannot be 
        changed at runtime. This method updates the GUI and prepares infrastructure for future 
        firmware command support.
        """
        # Update GUI threshold markers
        self._update_threshold_markers()
        
        # Note: TCP command transmission to firmware is not implemented in this version
        # because firmware threshold (Config::EDGE_THRESHOLD_VOLTS at line 163) is constexpr
        # Future enhancement: modify firmware to accept runtime threshold commands via TCP

    def _update_threshold_markers(self) -> None:
        """Update red threshold line position on IR sensor progress bars based on configured threshold."""
        threshold_voltage = self.ir_threshold_config.get()
        
        for idx, canvas in enumerate(self.sensor_threshold_markers):
            canvas.delete('all')  # Clear previous lines
            
            # Calculate pixel position: threshold_voltage / 4.096V * canvas_width
            # Use canvas.winfo_width() if rendered, otherwise use nominal 150px
            canvas_width = canvas.winfo_width() if canvas.winfo_width() > 1 else 150
            line_x = (threshold_voltage / 4.096) * canvas_width
            
            # Draw white outline first for visibility (halo effect when bar is red)
            canvas.create_line(line_x, 0, line_x, 20, fill='#FFFFFF', width=4, tags='threshold_outline')
            # Draw vertical red line (2px wide) on top
            canvas.create_line(line_x, 0, line_x, 20, fill='#FF0000', width=2, tags='threshold')
    
    def _on_threshold_marker_enter(self, event) -> None:
        """Change cursor to horizontal resize when hovering over threshold marker."""
        event.widget.config(cursor="sb_h_double_arrow")
    
    def _on_threshold_marker_leave(self, event) -> None:
        """Restore default cursor when leaving threshold marker."""
        event.widget.config(cursor="")
    
    def _on_threshold_drag_start(self, event) -> None:
        """Start dragging threshold marker."""
        # Store initial drag position (optional, for future enhancements)
        self._dragging = True
        return "break"  # Prevent event from bubbling to window drag handler
    
    def _on_threshold_drag_motion(self, event) -> None:
        """Update threshold value while dragging marker."""
        if not hasattr(self, '_dragging') or not self._dragging:
            return "break"
        
        # Get canvas width (use actual width if rendered, fallback to 150px)
        canvas = event.widget
        canvas_width = canvas.winfo_width() if canvas.winfo_width() > 1 else 150
        
        # Convert mouse X position to voltage (0-4.096V range)
        voltage = (event.x / canvas_width) * 4.096
        
        # Clamp voltage to valid range
        voltage = max(0.0, min(4.096, voltage))
        
        # Quantize to 0.1V steps for discrete movement
        voltage = round(voltage * 10) / 10
        
        # Update threshold config (this will automatically update Spinbox via DoubleVar)
        self.ir_threshold_config.set(voltage)
        
        # Redraw all threshold markers immediately
        self._update_threshold_markers()
        
        return "break"  # Prevent event from bubbling to window drag handler
    
    def _on_threshold_drag_end(self, event) -> None:
        """Finish dragging threshold marker."""
        self._dragging = False
        return "break"  # Prevent event from bubbling to window drag handler

    def _setup_window_drag(self) -> None:
        """Enable dragging the window by clicking and dragging on the title bar or background."""
        # Bind to the main window
        self.bind("<Button-1>", self._start_drag)
        self.bind("<B1-Motion>", self._on_drag)
        
    def _start_drag(self, event) -> None:
        """Record the starting position for window drag."""
        self._drag_start_x = event.x
        self._drag_start_y = event.y
        
    def _on_drag(self, event) -> None:
        """Move the window as the mouse is dragged."""
        x = self.winfo_x() + (event.x - self._drag_start_x)
        y = self.winfo_y() + (event.y - self._drag_start_y)
        self.geometry(f"+{x}+{y}")

    # ------------------------------------------------------------------
    # Connection handling
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Queue processing and UI updates
    # ------------------------------------------------------------------
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

            assert isinstance(item, TelemetryPacket)
            self._update_display(item)

        self._schedule_queue_processing()

    def _update_display(self, packet: TelemetryPacket) -> None:
        self.timestamp_var.set(f"{packet.timestamp}")
        self.status_var.set(time.strftime("Last update: %H:%M:%S"))
        
        # Display firmware threshold values
        self.firmware_ir_threshold_var.set(f"{packet.ir_edge_threshold:.2f}V")
        # Note: ToF detection threshold is not displayed in UI currently
        # Update IR sensor data with color-coded bars
        for idx in range(4):
            raw_value = packet.sensors_raw[idx] if idx < len(packet.sensors_raw) else "-"
            volt_value = packet.sensors_voltage[idx] if idx < len(packet.sensors_voltage) else "-"
            self.sensor_raw_labels[idx].set(str(raw_value))
            self.sensor_voltage_labels[idx].set(f"{volt_value:.3f}" if isinstance(volt_value, (float, int)) else str(volt_value))
            
            # Update progress bar value and color style based on voltage zones
            if isinstance(volt_value, (float, int)):
                bar_value = max(0.0, min(4.096, float(volt_value)))
                self.sensor_voltage_bars[idx]['value'] = bar_value
                
                # Determine color zone: Green (0-1.5V) → Yellow (1.5-2.5V) → Red (>2.5V)
                if volt_value < 1.5:
                    self.sensor_voltage_bars[idx].configure(style='IR.green.Horizontal.TProgressbar')
                elif volt_value < 2.5:
                    self.sensor_voltage_bars[idx].configure(style='IR.yellow.Horizontal.TProgressbar')
                else:
                    self.sensor_voltage_bars[idx].configure(style='IR.red.Horizontal.TProgressbar')
            else:
                self.sensor_voltage_bars[idx]['value'] = 0.0
                self.sensor_voltage_bars[idx].configure(style='IR.green.Horizontal.TProgressbar')

        # Update ToF sensor data with color-coded bars
        for idx in range(3):
            distance_value = packet.tof_distances[idx] if idx < len(packet.tof_distances) else "-"
            valid = packet.tof_valid[idx] if idx < len(packet.tof_valid) else False
            status = packet.tof_status[idx] if idx < len(packet.tof_status) else "-"
            
            if isinstance(distance_value, int) and valid:
                self.tof_distance_labels[idx].set(f"{distance_value} mm")
                self.tof_status_labels[idx].set("✓ Valid")
                # Update proximity bar (inverse: closer = more filled)
                # Clamp distance to 0-2000mm, then invert so bar fills as object approaches
                proximity_value = max(0, 2000 - min(2000, distance_value))
                self.tof_proximity_bars[idx]['value'] = proximity_value
                
                # Determine color zone based on actual distance (not proximity):
                # Green (>=1200mm far) → Yellow (400-1199mm medium) → Red (<400mm close)
                if distance_value >= 1200:
                    self.tof_proximity_bars[idx].configure(style='ToF.green.Horizontal.TProgressbar')
                elif distance_value >= 400:
                    self.tof_proximity_bars[idx].configure(style='ToF.yellow.Horizontal.TProgressbar')
                else:
                    self.tof_proximity_bars[idx].configure(style='ToF.red.Horizontal.TProgressbar')
            else:
                self.tof_distance_labels[idx].set("-")
                self.tof_status_labels[idx].set(f"Error ({status})" if status != "-" else "-")
                self.tof_proximity_bars[idx]['value'] = 0
                self.tof_proximity_bars[idx].configure(style='ToF.green.Horizontal.TProgressbar')
        
        # Update ToF object direction
        self.tof_object_direction_var.set(packet.tof_object_direction if packet.tof_object_direction else "-")

        self.action_var.set(packet.robot_state.get("action", "-"))
        self.edge_detected_var.set(str(packet.robot_state.get("edge_detected", "-")))
        self.edge_direction_var.set(packet.robot_state.get("edge_direction", "-"))
        self.danger_level_var.set(str(packet.robot_state.get("danger_level", "-")))

        self.core0_freq_var.set(str(packet.system_info.get("core0_freq", "-")))
        self.core1_freq_var.set(str(packet.system_info.get("core1_freq", "-")))
        self.wifi_rssi_var.set(str(packet.system_info.get("wifi_rssi", "-")))
        self.free_heap_var.set(str(packet.system_info.get("free_heap", "-")))

    # ------------------------------------------------------------------
    # Tkinter lifecycle hooks
    # ------------------------------------------------------------------
    def destroy(self) -> None:
        self._disconnect()
        super().destroy()


def main() -> None:
    app = BottleSumoViewer()
    app.mainloop()


if __name__ == "__main__":
    main()
