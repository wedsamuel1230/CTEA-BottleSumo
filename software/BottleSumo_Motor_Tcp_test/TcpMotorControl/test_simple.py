"""
Enhanced TCP test - sends command and waits, showing all activity
"""
import socket
import time

HOST = '192.168.42.1'
PORT = 5000

print("Connecting...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(1.0)  # Short timeout for recv checks
sock.connect((HOST, PORT))
print("Connected!")

# Send auth
cmd = '{"action": "auth", "token": "BottleSumo2025Secure"}\n'
print(f"Sending: {cmd.strip()}")
sock.sendall(cmd.encode('utf-8'))
print("Sent!")

# Wait and check for response
print("\nListening for response (30 seconds)...")
sock.settimeout(0.1)  # Non-blocking-ish

start = time.time()
buffer = b''

try:
    while time.time() - start < 30:
        try:
            data = sock.recv(1024)
            if data:
                buffer += data
                print(f"[{time.time()-start:.1f}s] Received {len(data)} bytes: {data}")
                if b'\n' in buffer:
                    print(f"\nComplete response: {buffer}")
                    break
        except socket.timeout:
            # No data yet
            elapsed = time.time() - start
            if int(elapsed) % 5 == 0 and elapsed - int(elapsed) < 0.1:
                print(f"  ... still waiting ({int(elapsed)}s) ...")
            continue
        except Exception as e:
            print(f"Error: {e}")
            break
finally:
    sock.close()
    
if not buffer:
    print("\n⚠️  NO RESPONSE RECEIVED")
    print("\nPossible issues:")
    print("1. Arduino code not uploaded or crashed")
    print("2. Serial Monitor shows errors")
    print("3. Command parser has a bug")
    print("\nCheck Arduino Serial Monitor (115200 baud) for:")
    print("  [WiFi] Client connected from ...")
    print("  [Command] Received: ...")
