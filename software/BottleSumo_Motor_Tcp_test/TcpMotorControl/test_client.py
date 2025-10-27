#!/usr/bin/env python3
"""
TCP Motor Control Test Script

Simple test client for validating the TCP motor control system.
Tests authentication, motor commands, status query, and error handling.

Usage:
    python test_client.py [--host 192.168.4.1] [--port 5000] [--token YOUR_TOKEN]
"""

import socket
import json
import time
import argparse
import sys

class MotorControlClient:
    def __init__(self, host='192.168.4.1', port=5000, token='BottleSumo2025Secure'):
        self.host = host
        self.port = port
        self.token = token
        self.sock = None
        
    def connect(self):
        """Establish TCP connection"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            print(f"✓ Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def send_command(self, cmd_dict):
        """Send JSON command and receive response"""
        try:
            cmd_json = json.dumps(cmd_dict)
            self.sock.sendall((cmd_json + '\n').encode('utf-8'))
            print(f"→ Sent: {cmd_json}")
            
            # Receive response (read until newline)
            response = b''
            while True:
                chunk = self.sock.recv(1)
                if not chunk or chunk == b'\n':
                    break
                response += chunk
            
            resp_json = json.loads(response.decode('utf-8'))
            print(f"← Received: {json.dumps(resp_json)}")
            return resp_json
        except Exception as e:
            print(f"✗ Command error: {e}")
            return None
    
    def close(self):
        """Close connection"""
        if self.sock:
            self.sock.close()
            print("✓ Connection closed")
    
    def test_auth(self):
        """Test authentication"""
        print("\n--- Test: Authentication ---")
        resp = self.send_command({"action": "auth", "token": self.token})
        if resp and resp.get("status") == "ok":
            print("✓ Authentication successful")
            return True
        else:
            print("✗ Authentication failed")
            return False
    
    def test_invalid_auth(self):
        """Test invalid authentication"""
        print("\n--- Test: Invalid Authentication ---")
        resp = self.send_command({"action": "auth", "token": "wrongtoken"})
        if resp and resp.get("status") == "error" and resp.get("code") == "AUTH_INVALID":
            print("✓ Invalid auth correctly rejected")
            return True
        else:
            print("✗ Invalid auth not handled correctly")
            return False
    
    def test_set_forward(self):
        """Test forward motion"""
        print("\n--- Test: Forward Motion ---")
        resp = self.send_command({"action": "set", "left": 150, "right": 150})
        if resp and resp.get("status") == "ok":
            print("✓ Forward motion command accepted")
            return True
        else:
            print("✗ Forward motion command failed")
            return False
    
    def test_set_reverse(self):
        """Test reverse motion"""
        print("\n--- Test: Reverse Motion ---")
        resp = self.send_command({"action": "set", "left": -150, "right": -150})
        if resp and resp.get("status") == "ok":
            print("✓ Reverse motion command accepted")
            return True
        else:
            print("✗ Reverse motion command failed")
            return False
    
    def test_stop(self):
        """Test stop command"""
        print("\n--- Test: Stop ---")
        resp = self.send_command({"action": "stop"})
        if resp and resp.get("status") == "ok":
            print("✓ Stop command accepted")
            return True
        else:
            print("✗ Stop command failed")
            return False
    
    def test_status(self):
        """Test status query"""
        print("\n--- Test: Status Query ---")
        resp = self.send_command({"action": "status"})
        if resp and resp.get("status") == "ok":
            print(f"  Auth: {resp.get('auth')}")
            print(f"  Motors: {resp.get('motors')}")
            print(f"  Sensors: {resp.get('sensors')}")
            print(f"  Safety: {resp.get('safety')}")
            print("✓ Status query successful")
            return True
        else:
            print("✗ Status query failed")
            return False
    
    def test_out_of_range(self):
        """Test out-of-range value rejection"""
        print("\n--- Test: Out of Range ---")
        resp = self.send_command({"action": "set", "left": 500, "right": 500})
        if resp and resp.get("status") == "error" and resp.get("code") == "OUT_OF_RANGE":
            print("✓ Out-of-range values correctly rejected")
            return True
        else:
            print("✗ Out-of-range handling failed")
            return False
    
    def test_no_auth_command(self):
        """Test command without authentication"""
        print("\n--- Test: No Auth Command ---")
        # Create new connection without auth
        new_client = MotorControlClient(self.host, self.port, self.token)
        if not new_client.connect():
            return False
        
        resp = new_client.send_command({"action": "set", "left": 100, "right": 100})
        new_client.close()
        
        if resp and resp.get("status") == "error" and resp.get("code") == "AUTH_REQUIRED":
            print("✓ Unauthenticated command correctly rejected")
            return True
        else:
            print("✗ Auth requirement not enforced")
            return False

def main():
    parser = argparse.ArgumentParser(description='Test TCP Motor Control System')
    parser.add_argument('--host', default='192.168.4.1', help='Robot IP address')
    parser.add_argument('--port', type=int, default=5000, help='TCP port')
    parser.add_argument('--token', default='BottleSumo2025Secure', help='Auth token')
    args = parser.parse_args()
    
    print("========================================")
    print("TCP Motor Control - Test Suite")
    print("========================================")
    
    client = MotorControlClient(args.host, args.port, args.token)
    
    if not client.connect():
        sys.exit(1)
    
    tests = [
        ("Invalid Auth", client.test_invalid_auth),
        ("Valid Auth", client.test_auth),
        ("Set Forward", client.test_set_forward),
        ("Stop", client.test_stop),
        ("Set Reverse", client.test_set_reverse),
        ("Stop", client.test_stop),
        ("Status Query", client.test_status),
        ("Out of Range", client.test_out_of_range),
        ("No Auth Command", client.test_no_auth_command),
    ]
    
    results = []
    for name, test_func in tests:
        try:
            # Add delay between tests
            time.sleep(0.5)
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"✗ Test '{name}' exception: {e}")
            results.append((name, False))
    
    client.close()
    
    # Summary
    print("\n========================================")
    print("Test Summary")
    print("========================================")
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status}: {name}")
    
    print(f"\nTotal: {passed}/{total} passed")
    
    if passed == total:
        print("\n✓ All tests passed!")
        sys.exit(0)
    else:
        print(f"\n✗ {total - passed} test(s) failed")
        sys.exit(1)

if __name__ == '__main__':
    main()
