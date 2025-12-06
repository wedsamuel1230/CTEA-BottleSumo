#!/usr/bin/env python3
"""
Quick authentication test for BottleSumo TCP server
Tests the auth command and response timing
"""

import socket
import json
import time

def test_auth():
    HOST = '192.168.42.1'
    PORT = 5000
    TOKEN = 'BottleSumo2025Secure'
    
    print("=" * 60)
    print("BottleSumo Authentication Test")
    print("=" * 60)
    
    try:
        # Create socket
        print(f"\n[1] Connecting to {HOST}:{PORT}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(15.0)
        
        start = time.time()
        sock.connect((HOST, PORT))
        connect_time = time.time() - start
        print(f"    ✓ Connected in {connect_time:.3f} seconds")
        
        # Send auth command
        print(f"\n[2] Sending auth command...")
        auth_cmd = {"action": "auth", "token": TOKEN}
        cmd_json = json.dumps(auth_cmd)
        print(f"    → {cmd_json}")
        
        start = time.time()
        sock.sendall((cmd_json + '\n').encode('utf-8'))
        send_time = time.time() - start
        print(f"    ✓ Sent in {send_time:.3f} seconds")
        
        # Receive response
        print(f"\n[3] Waiting for response...")
        response = b''
        start = time.time()
        
        while True:
            elapsed = time.time() - start
            if elapsed > 10.0:
                print(f"    ✗ Timeout after {elapsed:.3f} seconds")
                print(f"    Partial response: {response}")
                break
                
            try:
                chunk = sock.recv(1024)
                if not chunk:
                    print(f"    ✗ Connection closed by server")
                    break
                    
                response += chunk
                print(f"    ... received {len(chunk)} bytes (total: {len(response)})")
                
                if b'\n' in response:
                    response = response.split(b'\n')[0]
                    recv_time = time.time() - start
                    print(f"    ✓ Received in {recv_time:.3f} seconds")
                    break
                    
            except socket.timeout:
                print(f"    ... waiting ({elapsed:.1f}s elapsed)...")
                continue
        
        # Parse response
        if response:
            print(f"\n[4] Response:")
            print(f"    Raw: {response}")
            try:
                resp_json = json.loads(response.decode('utf-8'))
                print(f"    JSON: {json.dumps(resp_json, indent=2)}")
                
                if resp_json.get('status') == 'ok':
                    print(f"\n✓ AUTHENTICATION SUCCESSFUL!")
                else:
                    print(f"\n✗ Authentication failed: {resp_json}")
            except json.JSONDecodeError as e:
                print(f"    ✗ Invalid JSON: {e}")
        
        sock.close()
        
    except socket.timeout as e:
        print(f"\n✗ Socket timeout: {e}")
        print("   This usually means the server is not responding")
        print("   Check Serial Monitor for server activity")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "=" * 60)

if __name__ == "__main__":
    test_auth()
