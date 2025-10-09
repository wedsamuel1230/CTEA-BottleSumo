# Quick Start Guide - Bottle Sumo Dual-Core Snapshot

## Table of Contents
1. [5-Minute Quick Start](#5-minute-quick-start)
2. [Detailed Wiring Guide](#detailed-wiring-guide)
3. [Example Code Patterns](#example-code-patterns)
4. [Testing & Debugging](#testing--debugging)

---

## 5-Minute Quick Start

### Step 1: Install Required Libraries
Open Arduino IDE and install these libraries via Library Manager:
```
1. Adafruit ADS1X15  (for ADS1115 ADC)
2. VL53L1X           (for ToF distance sensors)
```

### Step 2: Board Configuration
1. **Tools** → **Board** → **Raspberry Pi Pico/RP2040** → **Raspberry Pi Pico W**
2. **Tools** → **Port** → Select your Pico W COM port
3. **Tools** → **Upload Method** → **Default (UF2)**

### Step 3: Wire Up Your Robot
Minimum viable wiring (IR sensors only):
```
ADS1115:
  VDD → 3.3V (Pin 36)
  GND → GND  (Pin 38)
  SDA → GP4  (Pin 6)
  SCL → GP5  (Pin 7)
  
  A0 → QRE1113 #1 (Front-Left)
  A1 → QRE1113 #2 (Front-Right)
  A2 → QRE1113 #3 (Back-Left)
  A3 → QRE1113 #4 (Back-Right)
```

### Step 4: Upload & Test
1. Open `BottleSumo_DualCore_Snapshot.ino`
2. Click **Upload** (Ctrl+U)
3. Open **Serial Monitor** (Ctrl+Shift+M)
4. Set baud rate to **115200**
5. You should see:
```
========================================
Bottle Sumo - Dual-Core Snapshot Architecture
========================================
Core 0: State machine and control
Core 1: Sensor acquisition

Waiting for Core 1 initialization...
Core 1: Sensor acquisition core starting...
Initializing ADS1115...
✓ ADS1115 initialized
...
✓ System ready
========================================
```

---

## Detailed Wiring Guide

### Complete System Wiring

#### Pin Map Summary
| Component | Pin | GPIO | Function |
|-----------|-----|------|----------|
| ADS1115 SDA | 6 | GP4 | I2C Data |
| ADS1115 SCL | 7 | GP5 | I2C Clock |
| ToF SDA | 31 | GP26 | I2C Data |
| ToF SCL | 32 | GP27 | I2C Clock |
| ToF Right XSHUT | 17 | GP13 | Shutdown Control |
| ToF Front XSHUT | 16 | GP12 | Shutdown Control |
| ToF Left XSHUT | 15 | GP11 | Shutdown Control |
| Button 1 | 5 | GP3 | Mode Select |
| Button 2 | 6 | GP4 | (Optional) |
| Button 3 | 7 | GP5 | (Optional) |
| Button 4 | 9 | GP6 | (Optional) |

#### Power Distribution
```
Battery/Power Supply:
  (+) → Motor Driver VIN
      → Buck Converter Input (if using)
      → Pico W VSYS (Pin 39)
  
  (-) → Common Ground
      → All GND pins on Pico W
      → ADS1115 GND
      → All ToF sensor GND
      → Motor Driver GND

3.3V Rail (from Pico W Pin 36):
  → ADS1115 VDD
  → All ToF sensor VDD
  → QRE1113 VCC (via 1kΩ resistor if needed)
```

#### QRE1113 IR Sensor Wiring (Each sensor)
```
QRE1113:
  Collector → 3.3V via 10kΩ pull-up
  Collector → ADS1115 A0/A1/A2/A3
  Emitter   → GND
```

#### VL53L1X ToF Sensor Wiring (Each sensor)
```
VL53L1X:
  VDD   → 3.3V
  GND   → GND
  SDA   → GP26 (shared I2C bus)
  SCL   → GP27 (shared I2C bus)
  XSHUT → GP11/12/13 (unique per sensor)
  GPIO1 → Not connected (optional interrupt)
```

### Wiring Diagram ASCII Art
```
                    Raspberry Pi Pico W
                   ┌─────────────────┐
                   │                 │
    ADS1115    SDA │ GP4      GP26   │ SDA   ToF Sensors
    I2C Bus    SCL │ GP5      GP27   │ SCL   I2C Bus
                   │                 │
                   │  GP11 ◄─────────┼──► ToF Left XSHUT
                   │  GP12 ◄─────────┼──► ToF Front XSHUT
                   │  GP13 ◄─────────┼──► ToF Right XSHUT
                   │                 │
    Button 1       │  GP3            │
    Button 2       │  GP4 (shared)   │
    Button 3       │  GP5 (shared)   │
    Button 4       │  GP6            │
                   │                 │
    3.3V ◄─────────┤  3V3 OUT        │
    GND  ◄─────────┤  GND            │
                   └─────────────────┘

    ADS1115              ToF Sensors (×3)
   ┌────────┐           ┌────────┐
   │ VDD  ──┼──→ 3.3V   │ VDD  ──┼──→ 3.3V
   │ GND  ──┼──→ GND    │ GND  ──┼──→ GND
   │ SDA  ──┼──→ GP4    │ SDA  ──┼──→ GP26
   │ SCL  ──┼──→ GP5    │ SCL  ──┼──→ GP27
   │ A0   ◄─┼─── QRE1   │ XSHUT ─┼──→ GP11/12/13
   │ A1   ◄─┼─── QRE2   └────────┘
   │ A2   ◄─┼─── QRE3
   │ A3   ◄─┼─── QRE4
   └────────┘
```

---

## Example Code Patterns

### Example 1: Modifying Edge Detection Threshold

In Core 0, to dynamically adjust edge thresholds:

```cpp
void adjustThreshold(int sensorIndex, float newThreshold) {
  float thresholds[4] = {newThreshold, 0, 0, 0};
  uint8_t mask = (1 << sensorIndex);  // Update only this sensor
  
  // Send command to Core 1
  updateMotorAndThresholds(
    g_commandBlock.motorLeft,   // Keep current motor values
    g_commandBlock.motorRight,
    thresholds,                 // New threshold
    mask,                       // Which sensor to update
    0                          // No flag changes
  );
  
  Serial.printf("Updated sensor %d threshold to %.2f V\n", 
                sensorIndex, newThreshold);
}

// Usage in loop():
void loop() {
  // Adjust front-left sensor threshold based on lighting
  if (lightingBright) {
    adjustThreshold(0, 3.0);  // Higher threshold for bright conditions
  } else {
    adjustThreshold(0, 2.0);  // Lower threshold for dim conditions
  }
  
  // ... rest of loop
}
```

### Example 2: Custom State - Spin Attack

Add a new aggressive state:

```cpp
enum RobotState {
  STATE_INIT,
  STATE_SEARCH,
  STATE_ATTACK,
  STATE_SPIN_ATTACK,  // NEW STATE
  STATE_RETREAT,
  STATE_EMERGENCY
};

void runStateMachine(const SensorSnapshot& snap) {
  // ... existing code ...
  
  // New transition logic
  if (snap.opponentDirMask == 0x07 && snap.dangerLevel == 0) {
    // All ToF sensors see opponent and we're safe
    nextState = STATE_SPIN_ATTACK;
  }
  
  // ... existing code ...
  
  // New state action
  switch (currentState) {
    // ... existing cases ...
    
    case STATE_SPIN_ATTACK:
      // Spin while moving forward for aggressive push
      updateMotorAndThresholds(100, 50, nullptr, 0, 0);
      
      // Exit after 2 seconds
      if (millis() - stateEntryTime > 2000) {
        currentState = STATE_ATTACK;
      }
      break;
  }
}
```

### Example 3: Button-Triggered Mode Change

Use buttons to switch between aggressive and defensive modes:

```cpp
enum BehaviorMode {
  MODE_DEFENSIVE,
  MODE_BALANCED,
  MODE_AGGRESSIVE
};

BehaviorMode currentMode = MODE_BALANCED;

void handleButtons(const SensorSnapshot& snap) {
  // Button edge detection - only trigger on button press (not hold)
  if (snap.buttonsEdgeMask & 0x01) {  // Button 1 edge
    if (snap.buttonsStableMask & 0x01) {  // Button 1 pressed
      currentMode = MODE_DEFENSIVE;
      Serial.println("Mode: DEFENSIVE");
    }
  }
  
  if (snap.buttonsEdgeMask & 0x02) {  // Button 2 edge
    if (snap.buttonsStableMask & 0x02) {  // Button 2 pressed
      currentMode = MODE_BALANCED;
      Serial.println("Mode: BALANCED");
    }
  }
  
  if (snap.buttonsEdgeMask & 0x04) {  // Button 3 edge
    if (snap.buttonsStableMask & 0x04) {  // Button 3 pressed
      currentMode = MODE_AGGRESSIVE;
      Serial.println("Mode: AGGRESSIVE");
    }
  }
}

void loop() {
  SensorSnapshot snap;
  fetchLatestSnapshot(snap);
  
  // Check for button presses
  handleButtons(snap);
  
  // Adjust behavior based on mode
  int16_t attackSpeed = 80;
  int16_t searchSpeed = 40;
  
  switch (currentMode) {
    case MODE_DEFENSIVE:
      attackSpeed = 60;
      searchSpeed = 20;
      break;
    case MODE_BALANCED:
      attackSpeed = 80;
      searchSpeed = 40;
      break;
    case MODE_AGGRESSIVE:
      attackSpeed = 100;
      searchSpeed = 60;
      break;
  }
  
  // Use mode-adjusted speeds in state machine
  runStateMachine(snap, attackSpeed, searchSpeed);
  
  // ... rest of loop
}
```

### Example 4: ToF-Based Opponent Tracking

Track opponent position accurately:

```cpp
void trackOpponent(const SensorSnapshot& snap) {
  if (snap.opponentDirMask == 0) {
    Serial.println("No opponent detected");
    return;
  }
  
  // Decode direction mask
  bool rightSees = snap.opponentDirMask & 0x01;
  bool frontSees = snap.opponentDirMask & 0x02;
  bool leftSees = snap.opponentDirMask & 0x04;
  
  // Get distances (only valid ones)
  uint16_t rightDist = (snap.tofValidMask & 0x01) ? snap.tofDist[0] : 9999;
  uint16_t frontDist = (snap.tofValidMask & 0x02) ? snap.tofDist[1] : 9999;
  uint16_t leftDist = (snap.tofValidMask & 0x04) ? snap.tofDist[2] : 9999;
  
  // Find closest opponent
  uint16_t minDist = min(rightDist, min(frontDist, leftDist));
  
  int16_t leftMotor = 0, rightMotor = 0;
  
  if (frontDist == minDist && frontSees) {
    // Opponent in front - charge!
    leftMotor = 100;
    rightMotor = 100;
    Serial.println("CHARGING FORWARD");
  }
  else if (rightDist == minDist && rightSees) {
    // Opponent on right - turn right
    leftMotor = 80;
    rightMotor = 40;
    Serial.println("TURNING RIGHT");
  }
  else if (leftDist == minDist && leftSees) {
    // Opponent on left - turn left
    leftMotor = 40;
    rightMotor = 80;
    Serial.println("TURNING LEFT");
  }
  
  // Send motor commands
  updateMotorAndThresholds(leftMotor, rightMotor, nullptr, 0, 0);
}
```

### Example 5: Data Logging

Log sensor snapshots for analysis:

```cpp
void logSnapshot(const SensorSnapshot& snap) {
  // CSV format for easy import to Excel/Python
  Serial.print(snap.sequence);
  Serial.print(",");
  Serial.print(snap.captureMillis);
  Serial.print(",");
  
  // IR voltages
  for (int i = 0; i < 4; i++) {
    Serial.print(snap.irVolts[i], 3);
    Serial.print(",");
  }
  
  // ToF distances
  for (int i = 0; i < 3; i++) {
    if (snap.tofValidMask & (1 << i)) {
      Serial.print(snap.tofDist[i]);
    } else {
      Serial.print("NA");
    }
    Serial.print(",");
  }
  
  // State
  Serial.print(snap.dangerLevel);
  Serial.print(",");
  Serial.print(snap.edgeDetected);
  Serial.println();
}

void loop() {
  static unsigned long lastLog = 0;
  
  SensorSnapshot snap;
  fetchLatestSnapshot(snap);
  
  // Log at 10 Hz
  if (millis() - lastLog >= 100) {
    logSnapshot(snap);
    lastLog = millis();
  }
  
  // ... rest of loop
}
```

---

## Testing & Debugging

### Test 1: Verify Core Communication

Check that Core 1 is publishing snapshots:

```cpp
void loop() {
  static uint32_t lastSeq = 0;
  
  SensorSnapshot snap;
  fetchLatestSnapshot(snap);
  
  if (snap.sequence != lastSeq) {
    Serial.printf("✓ New snapshot: seq=%lu age=%lu ms\n", 
                  snap.sequence, millis() - snap.captureMillis);
    lastSeq = snap.sequence;
  } else {
    Serial.println("⚠️ No new snapshots!");
  }
  
  delay(100);
}
```

Expected output (every 10ms):
```
✓ New snapshot: seq=1 age=2 ms
✓ New snapshot: seq=2 age=3 ms
✓ New snapshot: seq=3 age=2 ms
...
```

### Test 2: IR Sensor Calibration

Find optimal threshold values:

```cpp
void calibrateIRSensors() {
  Serial.println("=== IR Sensor Calibration ===");
  Serial.println("Place robot on BLACK surface, press button when ready");
  
  // Wait for button press (implement as needed)
  while (!buttonPressed()) delay(100);
  
  float blackValues[4] = {0};
  for (int i = 0; i < 100; i++) {
    SensorSnapshot snap;
    fetchLatestSnapshot(snap);
    for (int j = 0; j < 4; j++) {
      blackValues[j] += snap.irVolts[j];
    }
    delay(10);
  }
  
  Serial.println("\nBLACK readings (average):");
  for (int i = 0; i < 4; i++) {
    blackValues[i] /= 100.0;
    Serial.printf("Sensor %d: %.3f V\n", i, blackValues[i]);
  }
  
  Serial.println("\nPlace robot on WHITE surface, press button when ready");
  while (!buttonPressed()) delay(100);
  
  float whiteValues[4] = {0};
  for (int i = 0; i < 100; i++) {
    SensorSnapshot snap;
    fetchLatestSnapshot(snap);
    for (int j = 0; j < 4; j++) {
      whiteValues[j] += snap.irVolts[j];
    }
    delay(10);
  }
  
  Serial.println("\nWHITE readings (average):");
  for (int i = 0; i < 4; i++) {
    whiteValues[i] /= 100.0;
    Serial.printf("Sensor %d: %.3f V\n", i, whiteValues[i]);
  }
  
  Serial.println("\nRecommended thresholds (midpoint):");
  for (int i = 0; i < 4; i++) {
    float threshold = (blackValues[i] + whiteValues[i]) / 2.0;
    Serial.printf("Sensor %d: %.3f V\n", i, threshold);
  }
}
```

### Test 3: ToF Sensor Range Test

Verify ToF detection range:

```cpp
void testToFRange() {
  Serial.println("=== ToF Range Test ===");
  Serial.println("Move object in front of sensors");
  
  for (int i = 0; i < 100; i++) {
    SensorSnapshot snap;
    fetchLatestSnapshot(snap);
    
    Serial.print("R:");
    if (snap.tofValidMask & 0x01) {
      Serial.printf("%4d ", snap.tofDist[0]);
    } else {
      Serial.print(" --  ");
    }
    
    Serial.print("F:");
    if (snap.tofValidMask & 0x02) {
      Serial.printf("%4d ", snap.tofDist[1]);
    } else {
      Serial.print(" --  ");
    }
    
    Serial.print("L:");
    if (snap.tofValidMask & 0x04) {
      Serial.printf("%4d ", snap.tofDist[2]);
    } else {
      Serial.print(" --  ");
    }
    
    Serial.printf("| Opponent: 0x%02X\n", snap.opponentDirMask);
    delay(100);
  }
}
```

### Common Issues & Solutions

#### Issue: "❌ Failed to initialize ADS1115"
**Causes:**
- Wrong I2C address (default is 0x48)
- SDA/SCL wires swapped
- No pull-up resistors on I2C (Pico W has internal ones, should be OK)
- Power supply issue

**Debug:**
```cpp
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Scan I2C bus
  Serial.println("Scanning I2C bus...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Found device at 0x%02X\n", addr);
    }
  }
}
```

#### Issue: ToF sensors fail with "❌ LEFT ToF failed"
**Causes:**
- XSHUT pins not properly configured
- I2C address conflicts
- Insufficient power (ToF sensors draw ~20mA each)

**Solution:**
- Verify XSHUT wiring and add delays between sensor inits
- Use external 3.3V regulator if Pico W's regulator is insufficient
- Check with I2C scanner

#### Issue: Stale data warnings
**Causes:**
- Core 1 blocked or crashed
- I2C bus contention
- Sensor read taking too long

**Debug:**
Add Core 1 heartbeat:
```cpp
// In loop1():
static unsigned long lastHeartbeat = 0;
if (millis() - lastHeartbeat >= 1000) {
  Serial.println("Core 1: Heartbeat");
  lastHeartbeat = millis();
}
```

---

## Next Steps

1. **Test individual sensors** before full integration
2. **Calibrate IR sensors** on your specific arena surface
3. **Tune ToF detection threshold** for your opponent size
4. **Add motor control** hardware and integrate motor driver
5. **Implement PID control** for smooth movements
6. **Add telemetry** via WiFi for debugging

## Support

For issues or questions:
- Check existing examples in repository
- Review `BottleSumo_RealTime_Streaming.ino` for WiFi features
- Open issue on GitHub with detailed error messages

---

**Happy Bot Building! 🤖**
