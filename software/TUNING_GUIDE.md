# BottleSumo Robot Tuning Guide 🎯

## Quick Comparison of Three Implementations

| Feature | car_yoshi | car_tracking | car_ir |
|---------|-----------|--------------|---------|
| **Architecture** | Single-core | Dual-core | Single-core |
| **Primary Sensor** | ToF + IR | ToF only | IR only |
| **Edge Detection** | IR (4 sensors) | None | IR (4 sensors) |
| **Target Tracking** | ToF (5 sensors) | ToF (5 sensors) | None |
| **Search Speed** | 15°/s (slow) | 35°/s (fast) | Spin patterns |
| **Align Speed** | 10°/s (slow) | 32°/s (fast) | N/A |
| **Escape Speed** | 70% | N/A | 50% |
| **Complexity** | Medium | Low | Low |
| **Best For** | Competition | Testing ToF | Testing IR |

---

## 🎛️ car_yoshi Tuning Parameters

### Location: `software/car_yoshi/car_yoshi.ino`

```cpp
// ===== SPEED TUNING =====
constexpr float SEARCH_SPIN_SPEED = 15.0f;        // Currently TOO SLOW
constexpr float ALIGN_SPIN_SPEED = 10.0f;         // Currently TOO SLOW
constexpr float ALIGN_FINE_SPIN_SPEED = 8.0f;     // Currently TOO SLOW
constexpr float ESCAPE_SPEED = 70.0f;             // Edge escape speed (OK)
constexpr float BACK_ESCAPE_SPEED = 100.0f;       // Back edge escape (OK)

// ===== IR THRESHOLD TUNING =====
constexpr float IR_THRESHOLD_FRONT = 1.5f;        // Front sensors A1/A2
constexpr float IR_THRESHOLD_BACK = 2.5f;         // Back sensors A0/A3

// ===== TOF DETECTION RANGE =====
constexpr uint16_t DETECT_MIN_MM = 70;            // Min detection (70mm)
constexpr uint16_t DETECT_MAX_MM = 1000;          // Max detection (1m)
constexpr float BIAS_DEADZONE = 0.1f;             // Center deadzone

// ===== TIMING =====
constexpr uint32_t SENSOR_READ_INTERVAL_MS = 60;  // 16.7Hz sensor read
constexpr uint32_t LOST_HOLD_MS = 2000;           // Hold time after lost
```

### ⚡ RECOMMENDED CHANGES for car_yoshi:

```cpp
// MAKE IT COMPETITIVE (increase speeds to match car_tracking)
constexpr float SEARCH_SPIN_SPEED = 35.0f;        // ← Change from 15
constexpr float ALIGN_SPIN_SPEED = 28.0f;         // ← Change from 10
constexpr float ALIGN_FINE_SPIN_SPEED = 16.0f;    // ← Change from 8
```

**Why?** Your current speeds are TOO SLOW for competition:
- Search 15°/s → Takes 24 seconds for full rotation (too slow to find opponent)
- Align 10°/s → Takes forever to center on target
- Fine align 8°/s → Way too slow for dynamic tracking

---

## 🎛️ car_tracking Tuning Parameters

### Location: `software/car_tracking/car_tracking.ino`

```cpp
// ===== SPEED TUNING (PROVEN VALUES) =====
constexpr float SEARCH_SPIN_SPEED = 35.0f;        // ✓ GOOD
constexpr float ALIGN_SPIN_SPEED = 32.0f;         // ✓ GOOD
constexpr float ALIGN_FINE_SPIN_SPEED = 18.0f;    // ✓ GOOD

// ===== TOF DETECTION =====
constexpr uint16_t DETECT_MIN_MM = 70;
constexpr uint16_t DETECT_MAX_MM = 1000;
constexpr float BIAS_DEADZONE = 0.1f;

// ===== TIMING =====
constexpr uint32_t SENSOR_INTERVAL_MS = 60;       // 16.7Hz
constexpr float LOST_HOLD_MS = 2000.0f;
```

**car_tracking status:** ✅ **Already well-tuned**
- Fast search (35°/s) finds opponents quickly
- Aggressive alignment (32°/s coarse, 18°/s fine)
- Clean dual-core separation

---

## 🎛️ car_ir Tuning Parameters

### Location: `software/car_ir/car_ir.ino`

```cpp
// ===== IR THRESHOLD TUNING =====
float ir_threshold_front = 1.5F;                  // A1, A2 (front)
float ir_threshold_back = 3.0F;                   // A0, A3 (back) - HIGHER!

// ===== ESCAPE SPEEDS =====
const float ESCAPE_SPEED = 50.0F;                 // Could be faster
const float BACK_ESCAPE_SPEED = 1.0F;             // 100% forward
```

### ⚠️ Note about car_ir:
- Back threshold is **3.0V** (higher than car_yoshi's 2.5V)
- Escape speed only **50%** (could be faster)
- No ToF tracking (IR edge detection only)

---

## 🔧 TUNING PROCEDURE

### Step 1: Serial Monitor Test
```
Upload code → Open Serial Monitor (115200 baud)
Press START button → Watch output
```

### Step 2: Check Sensor Values (Before Start)
```
[IDLE] IR: BL=0.45V FL=0.38V FR=0.42V BR=0.51V
[IDLE] ToF: R45=850mm R23=----mm M0=654mm L23=----mm L45=723mm
```

**What to check:**
- ✅ IR values < 1.0V when on white surface
- ✅ IR values > 2.0V when over black line
- ✅ ToF shows distances when pointing at objects
- ❌ ToF shows "----" when no valid reading

### Step 3: Tune IR Thresholds
```cpp
// If IR not detecting edges:
1. Place robot near edge
2. Read IR values from serial
3. Adjust thresholds to be between white/black readings
4. Front sensors usually 1.5V, back sensors 2.5-3.0V
```

### Step 4: Tune Speeds

**For AGGRESSIVE play:**
```cpp
SEARCH_SPIN_SPEED = 40.0f;      // Very fast search
ALIGN_SPIN_SPEED = 35.0f;       // Fast alignment
ALIGN_FINE_SPIN_SPEED = 20.0f;  // Still responsive
```

**For BALANCED play (recommended):**
```cpp
SEARCH_SPIN_SPEED = 30.0f;      // Medium search
ALIGN_SPIN_SPEED = 25.0f;       // Moderate alignment
ALIGN_FINE_SPIN_SPEED = 15.0f;  // Smooth centering
```

**For DEFENSIVE play:**
```cpp
SEARCH_SPIN_SPEED = 20.0f;      // Slower search
ALIGN_SPIN_SPEED = 18.0f;       // Careful alignment
ALIGN_FINE_SPIN_SPEED = 10.0f;  // Very precise
```

### Step 5: Tune ToF Detection Range

**For AGGRESSIVE (early detection):**
```cpp
DETECT_MIN_MM = 50;             // Detect closer
DETECT_MAX_MM = 1200;           // Detect farther
```

**For CONSERVATIVE (reduce false positives):**
```cpp
DETECT_MIN_MM = 100;            // Ignore very close
DETECT_MAX_MM = 800;            // Ignore far objects
```

---

## 📊 Performance Comparison

### Current car_yoshi Performance:
- ⚠️ **TOO SLOW**: Search takes 24s for 360° rotation
- ⚠️ **TOO SLOW**: Alignment takes forever
- ✅ **GOOD**: Edge escape works well (70% speed)
- ✅ **GOOD**: Full-speed attack (100% forward)

### car_tracking Performance:
- ✅ **FAST**: Search completes 360° in 10.3s
- ✅ **RESPONSIVE**: Aligns quickly to targets
- ✅ **AGGRESSIVE**: 100% forward on centered target
- ❌ **NO EDGE**: No IR edge detection

### Recommended: car_yoshi with car_tracking speeds!

---

## 🚀 Quick Fix for car_yoshi

### Edit `software/car_yoshi/car_yoshi.ino`:

```cpp
// BEFORE (Lines 60-62):
constexpr float SEARCH_SPIN_SPEED = 15.0f;
constexpr float ALIGN_SPIN_SPEED = 10.0f;
constexpr float ALIGN_FINE_SPIN_SPEED = 8.0f;

// AFTER (RECOMMENDED):
constexpr float SEARCH_SPIN_SPEED = 35.0f;        // Match car_tracking
constexpr float ALIGN_SPIN_SPEED = 28.0f;         // Slightly slower than car_tracking
constexpr float ALIGN_FINE_SPIN_SPEED = 16.0f;    // Responsive fine tuning
```

---

## 🔍 Debugging Serial Port Issues

### Error: "Could not connect to /dev/cu.usbmodem1401"

**Solutions:**
1. **Close Serial Monitor before uploading**
   - Arduino IDE: Close Serial Monitor window
   - Upload code
   - Then reopen Serial Monitor

2. **Check USB cable**
   - Try different USB cable
   - Use USB 2.0 port (not USB-C hub)

3. **Reset Pico**
   - Hold BOOTSEL button
   - Press RESET button
   - Release both
   - Try upload again

4. **Check board settings:**
   ```
   Tools → Board → Raspberry Pi Pico/RP2040
   Tools → Port → /dev/cu.usbmodem1401
   Tools → Upload Method → Default (UF2)
   ```

5. **Manual upload (if above fails):**
   - Hold BOOTSEL, plug in USB
   - Pico appears as USB drive
   - Drag .uf2 file to drive

---

## 📈 Testing Procedure

### 1. IR Sensor Test (No ToF needed)
```cpp
// Temporarily disable tracking, just print IR values
void loop() {
    // Read IR
    int16_t raw[4];
    gAdc.readAll(raw, gIrVolts, 4);
    
    // Print IR values
    Serial.print("IR: ");
    for (int i = 0; i < 4; i++) {
        Serial.print(gIrVolts[i], 2);
        Serial.print(" ");
    }
    Serial.println();
    delay(100);
}
```

### 2. ToF Sensor Test
```cpp
// Check ToF readings
void loop() {
    gTof.readAll(gSamples, 70, 1000, 3);
    
    Serial.print("ToF: ");
    for (int i = 0; i < TOF_NUM; i++) {
        Serial.print(TOF_NAMES[i]);
        Serial.print("=");
        if (gSamples[i].valid) {
            Serial.print(gSamples[i].distanceMm);
        } else {
            Serial.print("----");
        }
        Serial.print("mm ");
    }
    Serial.println();
    delay(100);
}
```

### 3. Motor Test
```cpp
// Test motor directions
void loop() {
    // Forward
    gCar.setMotors(50, -50);  // Should go straight forward
    delay(1000);
    
    // Turn left
    gCar.setMotors(-30, -30);
    delay(1000);
    
    // Turn right
    gCar.setMotors(30, 30);
    delay(1000);
    
    gCar.stop();
    delay(2000);
}
```

---

## 🎯 Final Recommendations

### For Competition: Use car_yoshi WITH these changes:
1. ✅ Increase SEARCH_SPIN_SPEED to 35°/s (match car_tracking)
2. ✅ Increase ALIGN speeds (28°/s coarse, 16°/s fine)
3. ✅ Keep IR edge detection (already working)
4. ✅ Keep full-speed attack (100% forward)
5. ✅ Test IR thresholds on your specific dohyo surface

### For Testing ToF: Use car_tracking
- Already optimized
- Clean dual-core design
- No edge detection (good for testing ToF isolation)

### For Testing IR: Use car_ir
- Pure edge detection
- Simple single-purpose code
- Easy to tune thresholds

---

## 📋 Checklist Before Competition

- [ ] IR sensors detect ring edge reliably
- [ ] ToF sensors detect opponent at >50cm
- [ ] Robot searches full 360° in <15 seconds
- [ ] Robot aligns to centered target quickly
- [ ] Robot attacks at full speed when centered
- [ ] Edge escape completes in <1.5 seconds
- [ ] Serial monitor shows clear status messages
- [ ] Battery fully charged
- [ ] Buttons work (test + start)

---

## 🆘 Common Issues

### Issue: Robot too slow to find opponent
**Solution:** Increase `SEARCH_SPIN_SPEED` to 35-40°/s

### Issue: Robot overshoots target
**Solution:** Decrease `ALIGN_FINE_SPIN_SPEED` to 12-15°/s

### Issue: Robot doesn't detect ring edge
**Solution:** Tune `IR_THRESHOLD_FRONT/BACK` based on serial readings

### Issue: ToF sensors not detecting
**Solution:** Check I2C addresses with I2C scanner, verify wiring

### Issue: Motors don't respond
**Solution:** Check motor direction pins, verify Motor.h initialization

---

## 💡 Pro Tips

1. **Start conservative, increase speeds gradually**
2. **Test on actual dohyo surface** (lighting affects IR)
3. **Calibrate IR thresholds every time** (temperature/lighting changes)
4. **Keep Serial Monitor open** during testing for debugging
5. **Use car_tracking speeds** as baseline (they're proven)
6. **Back up working code** before making changes
7. **Test each subsystem individually** (IR, ToF, motors)

---

**Last Updated:** 2025-01-21
**Best Configuration:** car_yoshi with car_tracking speeds + IR edge detection
