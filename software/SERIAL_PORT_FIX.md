# 🚨 SERIAL PORT TIMEOUT FIX - Quick Reference

## Problem
```
Timeout. The IDE has not received the 'success' message from the monitor 
after successfully connecting to it. 
Could not connect to /dev/cu.usbmodem1401 serial port.
```

## ✅ SOLUTION - 5 Steps

### 1. **CLOSE Serial Monitor First** (Most Important!)
```
- In Arduino IDE: Close the Serial Monitor window
- In VS Code: Close any terminal with serial connection
- This is the #1 cause of upload failures
```

### 2. **Run the Fix Script**
```bash
cd /Users/wici/Documents/GitHub/CTEA-BottleSumo/software
./fix_serial_port.sh
```

### 3. **Check Arduino IDE Settings**
```
Tools → Board: "Raspberry Pi Pico" or "Raspberry Pi Pico W"
Tools → Port: /dev/cu.usbmodem1401
Tools → Upload Method: "Default (UF2)" or "Picotool"
```

### 4. **Upload Procedure**
```
1. Make sure Serial Monitor is CLOSED
2. Press Upload button (→)
3. Wait for "Upload complete" message
4. THEN open Serial Monitor
5. Set baud rate to 115200
```

### 5. **If Upload Still Fails - Manual Reset**
```
1. Unplug USB cable from Pico
2. Hold BOOTSEL button on Pico
3. While holding BOOTSEL, plug in USB cable
4. Release BOOTSEL button
5. Pico appears as USB drive "RPI-RP2"
6. Try upload again
```

---

## 🎯 Changes Made to car_yoshi

### Speed Improvements (Lines 60-62)
```cpp
// OLD (TOO SLOW):
constexpr float SEARCH_SPIN_SPEED = 15.0f;
constexpr float ALIGN_SPIN_SPEED = 10.0f;
constexpr float ALIGN_FINE_SPIN_SPEED = 10.0f;

// NEW (COMPETITIVE):
constexpr float SEARCH_SPIN_SPEED = 35.0f;        // 2.3x faster search
constexpr float ALIGN_SPIN_SPEED = 28.0f;         // 2.8x faster align
constexpr float ALIGN_FINE_SPIN_SPEED = 16.0f;    // 1.6x faster fine-tune
```

### Impact:
- **Search time**: 24 seconds → **10.3 seconds** for 360°
- **Alignment**: Much more responsive to targets
- **Competition ready**: Matches car_tracking proven speeds

---

## 🔍 Troubleshooting Checklist

### Before Upload:
- [ ] Serial Monitor is CLOSED
- [ ] No other programs using /dev/cu.usbmodem1401
- [ ] USB cable is connected directly (not through hub)
- [ ] Board selected: Raspberry Pi Pico (W)
- [ ] Port selected: /dev/cu.usbmodem1401

### During Upload:
- [ ] Green LED on Pico should blink during upload
- [ ] Progress bar shows upload percentage
- [ ] Wait for "Upload complete" message

### After Upload:
- [ ] Open Serial Monitor (115200 baud)
- [ ] Press RESET button on Pico
- [ ] Should see setup messages
- [ ] Should see sensor readings

---

## 🆘 If Nothing Works

### Last Resort - UF2 Upload:
```
1. Hold BOOTSEL button
2. Plug in USB cable (while holding BOOTSEL)
3. Release BOOTSEL
4. Pico appears as USB drive "RPI-RP2"
5. In Arduino IDE:
   Sketch → Export Compiled Binary
6. Find .uf2 file in sketch folder
7. Drag .uf2 file to RPI-RP2 drive
8. Pico automatically reboots and runs code
```

### Find .uf2 file location:
```bash
# After "Export Compiled Binary":
cd ~/Documents/Arduino/car_yoshi/build
ls -la *.uf2
```

---

## 📱 Quick Commands

### Check if port exists:
```bash
ls -la /dev/cu.usb*
```

### Check if port is in use:
```bash
lsof /dev/cu.usbmodem1401
```

### Kill process using port:
```bash
sudo lsof -t /dev/cu.usbmodem1401 | xargs sudo kill -9
```

### Test port communication:
```bash
screen /dev/cu.usbmodem1401 115200
# Press Ctrl+A then K to exit
```

---

## ✅ Success Indicators

When upload works correctly, you'll see:
```
Sketch uses XXXXX bytes (XX%) of program storage space.
Global variables use XXXXX bytes (XX%) of dynamic memory.
Resetting COM port...
Forcing reset using 1200bps open/close on port /dev/cu.usbmodem1401
PORTS {} / {} => {}
PORTS {} / {} => {}
Upgrading upload via picotool
Wrote XXXXX bytes to board
Upload complete
```

When Serial Monitor works, you'll see:
```
=== BottleSumo Car Yoshi ===

[SETUP] Initializing I2C...
[SETUP] ✓ I2C ready
[SETUP] ✓ ToF sensors online: 5/5
  - R45 @ 0x30
  - R23 @ 0x31
  ...
[IDLE] IR: BL=0.45V FL=0.38V ...
```

---

**Last Updated:** 2025-01-21
**File:** `/Users/wici/Documents/GitHub/CTEA-BottleSumo/software/car_yoshi/car_yoshi.ino`
**Speeds:** SEARCH=35°/s, ALIGN=28°/s, FINE=16°/s ✅
