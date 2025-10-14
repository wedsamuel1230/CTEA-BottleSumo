# Bottle Sumo Real-Time Viewer

A lightweight Tkinter GUI that connects to the Bottle Sumo robot's real-time TCP streaming server (port `4242`) and displays the telemetry JSON feed.

## Features

- Adjustable host/port connection controls
- Live sensor readouts (raw ADC values and converted voltages)
- Robot state summary (action, edge detection, danger level)
- System status (core loop frequencies, RSSI, free heap)
- Automatic reconnection feedback and error handling

## Prerequisites

- Python 3.10+
- The robot firmware running and exposing the TCP stream (default `:4242`)

Optional (recommended) tooling:
- [`uv`](https://github.com/astral-sh/uv) for fast virtual environment creation and package installation.

## Quick Start

```powershell
# From the repo root
cd realtime_monitor

# (Optional) create a virtual environment with uv
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
	Write-Error "uv not found. Install with: pip install uv"
}
else {
	uv venv
	.\.venv\Scripts\Activate.ps1
}

# Install dependencies (standard library only, so nothing to install)

# Run the viewer
python viewer.py
```

## Usage Tips

1. Ensure the robot firmware is already streaming JSON on the selected host/port.
2. Launch the viewer (`python viewer.py`) and click **Connect**.
3. The latest telemetry packet updates the display automatically.
4. Click **Disconnect** to stop the background reader thread.

If the connection fails, confirm the IP address, firewall settings, and that the robot is reachable from your machine.
