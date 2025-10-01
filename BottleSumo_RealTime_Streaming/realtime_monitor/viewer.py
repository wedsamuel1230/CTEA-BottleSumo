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
DEFAULT_HOST = "127.0.0.1"
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
    robot_state: Dict[str, Any] = field(default_factory=dict)
    system_info: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_json(cls, payload: Dict[str, Any]) -> "TelemetryPacket":
        """Create a packet from the decoded JSON dictionary."""
        sensors = payload.get("sensors", {})
        return cls(
            timestamp=payload.get("timestamp", 0),
            sensors_raw=list(sensors.get("raw", [])),
            sensors_voltage=list(sensors.get("voltage", [])),
            robot_state=dict(payload.get("robot_state", {})),
            system_info=dict(payload.get("system_info", {})),
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
        self.geometry("520x550")
        self.minsize(500, 550)

        self._reader_thread: Optional[TelemetryReader] = None
        self._queue: "queue.Queue[TelemetryPacket | Exception]" = queue.Queue()

        self._build_ui()
        self._schedule_queue_processing()

    # ------------------------------------------------------------------
    # UI construction helpers
    # ------------------------------------------------------------------
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

        ttk.Label(sensors_frame, text="Index").grid(row=0, column=0, padx=4, sticky="w")
        ttk.Label(sensors_frame, text="Raw").grid(row=0, column=1, padx=4, sticky="w")
        ttk.Label(sensors_frame, text="Voltage (V)").grid(row=0, column=2, padx=4, sticky="w")

        self.sensor_raw_labels: list[tk.StringVar] = []
        self.sensor_voltage_labels: list[tk.StringVar] = []
        for idx in range(4):
            ttk.Label(sensors_frame, text=f"Sensor {idx}").grid(row=idx + 1, column=0, sticky="w", padx=5)
            raw_var = tk.StringVar(value="-")
            volt_var = tk.StringVar(value="-")
            ttk.Label(sensors_frame, textvariable=raw_var).grid(row=idx + 1, column=1, sticky="w", padx=5)
            ttk.Label(sensors_frame, textvariable=volt_var).grid(row=idx + 1, column=2, sticky="w", padx=5)
            self.sensor_raw_labels.append(raw_var)
            self.sensor_voltage_labels.append(volt_var)

        # Robot state --------------------------------------------------------
        state_frame = ttk.LabelFrame(root, text="Robot State", padding=8)
        state_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
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
        system_frame.grid(row=3, column=0, sticky="ew", pady=(10, 0))
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
        status_bar.grid(row=4, column=0, sticky="ew", pady=(10, 0))

        root.rowconfigure(1, weight=1)

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

        for idx in range(4):
            raw_value = packet.sensors_raw[idx] if idx < len(packet.sensors_raw) else "-"
            volt_value = packet.sensors_voltage[idx] if idx < len(packet.sensors_voltage) else "-"
            self.sensor_raw_labels[idx].set(str(raw_value))
            self.sensor_voltage_labels[idx].set(f"{volt_value:.3f}" if isinstance(volt_value, (float, int)) else str(volt_value))

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
