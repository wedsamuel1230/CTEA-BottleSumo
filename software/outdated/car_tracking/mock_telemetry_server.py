#!/usr/bin/env python3
"""
Mock TCP Server for Testing Telemetry Viewer

Simulates Pico W car tracker telemetry stream for testing the GUI
without needing the actual hardware.

Usage:
    python3 mock_telemetry_server.py

Then connect telemetry_viewer.py to localhost:8080
"""

import socket
import json
import time
import math
import random

def generate_mock_telemetry(t):
    """Generate realistic mock telemetry data"""
    
    # Simulate distance oscillating between 200-600mm
    distance = int(400 + 200 * math.sin(t * 0.5))
    
    # Simulate bias (steering) following distance
    bias = math.sin(t * 0.3) * 0.8
    
    # Speed based on distance
    if distance < 200:
        speed = -20.0
    elif distance < 300:
        speed = 30.0
    else:
        speed = 50.0 + (distance - 300) * 0.1
    
    # Motor speeds with differential steering
    left_speed = speed * (1.0 - abs(bias) * 0.5) if bias < 0 else speed
    right_speed = speed * (1.0 - abs(bias) * 0.5) if bias > 0 else speed
    
    # Simulate 5 sensors (some random noise)
    sensors = {
        'R45': 0 if random.random() < 0.3 else random.randint(400, 800),
        'R23': 0 if random.random() < 0.2 else random.randint(300, 700),
        'M0': distance,
        'L23': 0 if random.random() < 0.2 else random.randint(300, 700),
        'L45': 0 if random.random() < 0.3 else random.randint(400, 800),
    }
    
    timestamp = int(time.time() * 1000) % 100000
    
    return {
        "dist": distance,
        "bias": round(bias, 2),
        "speed": round(speed, 1),
        "left": round(left_speed, 1),
        "right": round(right_speed, 1),
        "R45": sensors['R45'],
        "R23": sensors['R23'],
        "M0": sensors['M0'],
        "L23": sensors['L23'],
        "L45": sensors['L45'],
        "timestamp": timestamp
    }

def main():
    """Run mock TCP server"""
    host = 'localhost'
    port = 8080
    
    # Create TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(5)
    
    print(f"[INFO] Mock telemetry server listening on {host}:{port}")
    print("[INFO] Connect telemetry_viewer.py to localhost:8080")
    print("[INFO] Press Ctrl+C to stop")
    
    try:
        while True:
            # Accept client connection
            client, addr = server.accept()
            print(f"[INFO] Client connected: {addr}")
            
            try:
                t = 0
                while True:
                    # Generate and send mock telemetry
                    telemetry = generate_mock_telemetry(t)
                    json_str = json.dumps(telemetry) + '\n'
                    client.send(json_str.encode('utf-8'))
                    
                    # 10Hz update rate (100ms)
                    time.sleep(0.1)
                    t += 0.1
                    
            except (BrokenPipeError, ConnectionResetError):
                print(f"[INFO] Client disconnected: {addr}")
            except KeyboardInterrupt:
                break
            finally:
                client.close()
    
    except KeyboardInterrupt:
        print("\n[INFO] Server stopped")
    finally:
        server.close()

if __name__ == "__main__":
    main()
