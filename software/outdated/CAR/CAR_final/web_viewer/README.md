# BottleSumo Web Viewer (iPad Compatible)

This is a web-based viewer for the BottleSumo robot, designed to be used on an iPad or any device with a web browser.

## Prerequisites

You need Python installed on your computer.

## Installation

1.  Navigate to this folder in your terminal:
    ```bash
    cd software/gui/web_viewer
    ```

2.  Install the required Python packages:
    ```bash
    pip install -r requirements.txt
    ```

## Usage

1.  Start the web server:
    ```bash
    python app.py
    ```

2.  Find your computer's IP address on the local network (e.g., `192.168.1.x`).

3.  On your iPad (connected to the same WiFi network), open Safari and go to:
    ```
    http://<YOUR_COMPUTER_IP>:5000
    ```
    Replace `<YOUR_COMPUTER_IP>` with your actual IP address.

4.  In the web interface:
    *   Enter the Robot's IP address (default `192.168.4.1` if connected to robot's AP, or the robot's IP if on the same router).
    *   Click **Connect**.
    *   You should see telemetry data and be able to control the robot.

## Features

*   **Real-time Telemetry:** View IR and ToF sensor data.
*   **Motor Control:** Sliders to control motors (enable transmission first).
*   **Test Modes:** Switch between AUTO, TEST_MOTOR, etc.
*   **Thresholds:** Adjust IR sensor thresholds dynamically.