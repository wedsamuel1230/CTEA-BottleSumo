# Upload and Test Instructions

## Changes Made
✅ **Serial baud fixed**: 9600 → 115200  
✅ **Blocking delay reduced**: 2000ms → 500ms (faster boot)  
✅ **Added Serial.flush()**: Ensures setup messages are sent  
✅ **Added loop counter**: Proves main loop is running  
✅ **Added verbose logging**: Shows exact data flow for debugging  
✅ **Added client.flush()**: Ensures responses are sent immediately  

---

## Step 1: Upload Code to Pico W

1. Open Arduino IDE
2. Select **Tools → Board → Raspberry Pi Pico W**
3. Select **Tools → Port** → Your Pico W COM port
4. Click **Upload** (or Ctrl+U)
5. Wait for "Done uploading" message

---

## Step 2: Open Serial Monitor

1. Click **Tools → Serial Monitor** (or Ctrl+Shift+M)
2. **CRITICAL**: Set baud rate to **115200** (bottom right dropdown)
3. You should see:

```
========================================
TCP Motor Control - Initializing
========================================

[Setup] Initializing motors...
[MotorController] Initialized L/R motors
[Setup] Initializing ADS1115 sensors...
[Setup] Sensors ready. Run calibration via TCP command.
[CommandParser] Auth token configured
[Setup] Starting WiFi AP: BottleSumo_AP
[Setup] AP IP: 192.168.42.1
[Setup] TCP server listening on port 5000

[Setup] Initialization complete!
========================================

[Heartbeat] Loops=1023 Auth=N Motors=0,0 Sensors=0x0 Safe=Y
[Heartbeat] Loops=2047 Auth=N Motors=0,0 Sensors=0x0 Safe=Y
```

**CRITICAL CHECKS:**
- ✅ You see "Initialization complete!" → Setup succeeded
- ✅ You see "Loops=" increasing → Main loop is running
- ✅ No errors or crashes

---

## Step 3: Test Authentication

In PowerShell (in TcpMotorControl directory):

```powershell
python test_auth.py
```

**Expected Output:**

```
[1] Connecting to 192.168.42.1:5000...
    ✓ Connected in 0.002 seconds

[2] Sending auth command...
    → {"action": "auth", "token": "BottleSumo2025Secure"}
    ✓ Sent in 0.000 seconds

[3] Waiting for response...
    ✓ Response received in 0.045 seconds
    ← {"status":"ok"}

✓ Authentication successful!
```

**AND in Serial Monitor you should see:**

```
[WiFi] Client connected from 192.168.42.16
[WiFi] Processing command (47 bytes)
[Command] Received: {"action": "auth", "token": "BottleSumo2025Secure"}
[Command] AUTH successful
[WiFi] Sending response: {"status":"ok"}
```

---

## Step 4: Diagnosis If It Still Fails

### Scenario A: No serial output at all
**Problem**: Wrong baud rate or code not uploaded  
**Fix**: 
1. Double-check Serial Monitor is set to **115200 baud**
2. Try pressing **BOOTSEL** button and re-upload
3. Check if correct port is selected

### Scenario B: Setup completes but still timeout
**Problem**: Main loop not running (likely sensors.begin() hanging)  
**Evidence**: Serial shows "Initializing ADS1115 sensors..." but never shows "Initialization complete!"  
**Fix**: See SENSOR_DEBUG.md for workaround

### Scenario C: Setup completes, loop runs, but no response
**Problem**: Response generation or sending failing  
**Evidence**: Serial shows:
```
[WiFi] Client connected from 192.168.42.16
[WiFi] Processing command (47 bytes)
[Command] Received: {"action": "auth", "token": "BottleSumo2025Secure"}
[Command] AUTH successful
[WiFi] ERROR: Cannot send response - client not connected
```
**Fix**: Client disconnected before response sent - need to investigate timing

### Scenario D: Response sent but client doesn't receive
**Problem**: Network layer issue  
**Evidence**: Serial shows "Sending response: ..." but client times out  
**Fix**: Check WiFi connection stability, try shorter response

---

## Step 5: Full Functional Test

If auth works, test motor control:

```powershell
python test_client_gui.py
```

1. Enter IP: 192.168.42.1
2. Enter Port: 5000
3. Click **Connect**
4. Should see "Connected" and "Authenticated" messages
5. Move motor sliders - motors should respond
6. Serial Monitor should show command processing

---

## Rollback Instructions

If the new code causes problems:

```bash
git checkout HEAD -- TcpMotorControl.ino
```

Or manually revert these lines:
- Line 75: Change `Serial.begin(115200);` back to `Serial.begin(9600);`
- Line 76: Change `delay(500);` back to `delay(2000);`
- Remove lines with loopCounter
- Remove verbose logging in sendResponse and handleClient

---

## Success Criteria

✅ Serial Monitor shows "Initialization complete!" at 115200 baud  
✅ Heartbeat shows Loops= counter increasing  
✅ test_auth.py completes in <1 second with "Authentication successful!"  
✅ Serial Monitor shows all command/response logging  
✅ No timeout errors  

---

## Report Back

Please copy and paste:
1. **Full Serial Monitor output** from boot to first heartbeat
2. **Full test_auth.py output**
3. **Any error messages**

This will help me diagnose any remaining issues!
