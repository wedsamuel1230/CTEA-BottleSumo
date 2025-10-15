# GUI Layout Update - Two-Column Design

## New Layout Structure

The GUI has been reorganized into a more compact two-column layout for better space utilization.

### Layout Overview

```
┌────────────────────────────────────────────────────────────────────────────┐
│                    Bottle Sumo Real-Time Viewer                            │
│                          (900x950 window)                                  │
├────────────────────────────────────┬───────────────────────────────────────┤
│ ROW 0 - LEFT COLUMN                │ ROW 0 - RIGHT COLUMN                  │
│ ┌────────────────────────────────┐ │ ┌───────────────────────────────────┐ │
│ │     Connection                 │ │ │        Test Mode                  │ │
│ │  [Host] [Port] [Connect]       │ │ │  Mode: [Dropdown▼] [Apply]       │ │
│ │                                │ │ │  Current: AUTO                    │ │
│ └────────────────────────────────┘ │ └───────────────────────────────────┘ │
├────────────────────────────────────┼───────────────────────────────────────┤
│ ROW 1 - LEFT COLUMN                │ ROW 1 - RIGHT COLUMN                  │
│ ┌────────────────────────────────┐ │ ┌───────────────────────────────────┐ │
│ │     Sensors (IR)               │ │ │      Motor Control                │ │
│ │  Sensor 0  Raw | V | ▓▓▓░      │ │ │  Motor 1: [======|======]  50    │ │
│ │  Sensor 1  Raw | V | ▓▓░░      │ │ │  Motor 2: [======|======] -30    │ │
│ │  Sensor 2  Raw | V | ▓░░░      │ │ │  ☑ Enable Transmission           │ │
│ │  Sensor 3  Raw | V | ▓▓▓▓      │ │ │  Status: ✓ Active | M1:50 M2:-30│ │
│ │                                │ │ │                                   │ │
│ └────────────────────────────────┘ │ └───────────────────────────────────┘ │
├────────────────────────────────────┴───────────────────────────────────────┤
│ ROW 2 - FULL WIDTH                                                         │
│ ┌──────────────────────────────────────────────────────────────────────┐   │
│ │                    Threshold Configuration                           │   │
│ │  Sensor 0: [2.50] [Apply]   Sensor 1: [2.50] [Apply]               │   │
│ │  Sensor 2: [2.50] [Apply]   Sensor 3: [2.50] [Apply]               │   │
│ │  Firmware IR Thresholds: 2.50V, 2.50V, 2.50V, 2.50V                │   │
│ └──────────────────────────────────────────────────────────────────────┘   │
├────────────────────────────────────────────────────────────────────────────┤
│ ROW 3 - FULL WIDTH                                                         │
│ ┌──────────────────────────────────────────────────────────────────────┐   │
│ │                    ToF Sensors (VL53L0X)                             │   │
│ │  Right: 1200mm ✓ Valid  ▓▓▓░░                                       │   │
│ │  Front:  500mm ✓ Valid  ▓▓▓▓▓                                       │   │
│ │  Left:   800mm ✓ Valid  ▓▓▓▓░                                       │   │
│ │  Object Direction: Front                                             │   │
│ └──────────────────────────────────────────────────────────────────────┘   │
├────────────────────────────────────────────────────────────────────────────┤
│ ROW 4 - FULL WIDTH                                                         │
│ ┌──────────────────────────────────────────────────────────────────────┐   │
│ │                         Robot State                                  │   │
│ │  Action: ATTACK_FORWARD              Edge Detected: False           │   │
│ │  Edge Direction: -                   Danger Level: -                │   │
│ └──────────────────────────────────────────────────────────────────────┘   │
├────────────────────────────────────────────────────────────────────────────┤
│ ROW 5 - FULL WIDTH                                                         │
│ ┌──────────────────────────────────────────────────────────────────────┐   │
│ │                         System Info                                  │   │
│ │  Timestamp: 123456 ms               WiFi RSSI: -45 dBm              │   │
│ │  Core0 freq: 100 Hz                 Free heap: 123456 bytes         │   │
│ │  Core1 freq: 2.7 Hz                                                  │   │
│ └──────────────────────────────────────────────────────────────────────┘   │
├────────────────────────────────────────────────────────────────────────────┤
│ ROW 6 - FULL WIDTH (Status Bar)                                           │
│  Last update: 12:34:56                                                     │
└────────────────────────────────────────────────────────────────────────────┘
```

## Benefits of Two-Column Layout

### ✅ Advantages

1. **More Compact:** Width increased to 900px, height reduced to 950px (from 650x1000)
2. **Better Organization:** Related controls grouped side-by-side
3. **Logical Flow:** 
   - Top row: Connection & Mode control (quick access)
   - Second row: Sensors & Motors (real-time monitoring & control)
   - Lower rows: Configuration, additional sensors, state info
4. **Screen Real Estate:** Better use of horizontal space on modern widescreen monitors
5. **Quick Testing:** Motor controls right next to sensor readings for immediate feedback

### 🎯 Layout Structure

```
Row 0: [Connection (Left)]          [Test Mode (Right)]
Row 1: [Sensors (Left)]             [Motor Control (Right)]
Row 2: [Threshold Configuration (Full Width)]
Row 3: [ToF Sensors (Full Width)]
Row 4: [Robot State (Full Width)]
Row 5: [System Info (Full Width)]
Row 6: [Status Bar (Full Width)]
```

### 📐 Column Configuration

- **Left Column:** Weight=1, expandable
- **Right Column:** Weight=1, expandable
- **Row 1:** Weight=1 (Sensors/Motors row is expandable)
- **Full-width sections:** columnspan=2

### 🖥️ Window Dimensions

- **Size:** 900x950 pixels (wider, slightly shorter)
- **Minimum:** 850x900 pixels
- **Previous:** 650x1000 pixels

## Usage Notes

1. The two-column layout makes it easy to:
   - Connect and set test mode at a glance
   - Monitor sensors while adjusting motor controls
   - See immediate feedback when testing motors

2. The layout automatically adjusts:
   - Columns resize proportionally
   - Full-width sections remain centered
   - Expandable row (sensors/motors) grows with window

3. Perfect for testing workflow:
   - **Top:** Connect → Set Test Mode
   - **Middle:** Watch Sensors → Adjust Motors
   - **Bottom:** Monitor State & System

---

*Layout Update: October 16, 2025*
*Two-column design for improved usability*
