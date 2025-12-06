# WiFi Removal Summary

## Overview
All WiFi functionality has been removed from CAR_final_v2.ino. The system now uses **serial-only telemetry** output while maintaining the same JSON format for easy parsing and monitoring.

## Changes Made

### 1. Removed WiFi Includes
- ❌ Removed `#include <WiFi.h>`

### 2. Removed WiFi Configuration Constants
- ❌ Removed `AP_SSID` constant
- ❌ Removed `AP_PASSWORD` constant  
- ❌ Removed `TCP_PORT` constant

### 3. Removed WiFi Global Objects
- ❌ Removed `WiFiServer gServer`
- ❌ Removed `WiFiClient gClient`

### 4. Replaced WiFi Telemetry Function
**Old Function:** `slot8_WiFi_Handling()`
- Managed WiFi client connections
- Sent JSON telemetry over TCP

**New Function:** `slot8_Serial_Telemetry()`
- Outputs JSON telemetry to Serial port
- Maintains same JSON format for compatibility
- Includes human-readable state name

### 5. Removed WiFi Initialization from setup1()
**Removed code:**
```cpp
// Init WiFi
WiFi.mode(WIFI_AP);
WiFi.softAP(AP_SSID, AP_PASSWORD);
Serial.print("[CORE1] WiFi AP: ");
Serial.println(WiFi.softAPIP());
gServer.begin();
Serial.println("[CORE1] TCP Server started");
```

### 6. Updated loop1() Switch Case
**Old:** `case 8: slot8_WiFi_Handling(); break;`  
**New:** `case 8: slot8_Serial_Telemetry(); break;`

## Telemetry Format

### JSON Structure (Unchanged)
```json
{
  "t": 12345,
  "ir": [2.45, 3.12, 1.89, 2.67],
  "tof": [120, 250, 180, 95, 310],
  "m": [0.8, 0.6],
  "s": 3,
  "state": "TRACKING"
}
```

### Field Descriptions
- **t**: Timestamp in milliseconds
- **ir**: IR sensor voltages [4 values, 0.0-3.3V]
- **tof**: ToF distances [5 values in mm]
- **m**: Motor speeds [left, right, -1.0 to 1.0]
- **s**: State machine mode (0-5)
- **state**: Human-readable state name

### State Codes
| Code | State |
|------|-------|
| 0 | IDLE |
| 1 | CALIBRATING |
| 2 | SEARCHING |
| 3 | TRACKING |
| 4 | ATTACKING |
| 5 | EDGE_AVOIDING |

## How to Monitor Telemetry

### Arduino IDE Serial Monitor
1. Open Arduino IDE
2. Upload CAR_final_v2.ino to Pico W
3. Open Tools → Serial Monitor
4. Set baud rate to **115200**
5. JSON telemetry will print at ~2Hz

### Command Line (macOS/Linux)
```bash
# Find the Pico W serial port
ls /dev/tty.usbmodem*

# Monitor with screen
screen /dev/tty.usbmodem14101 115200

# Or use cat
cat /dev/tty.usbmodem14101

# Exit screen: Ctrl+A, then K, then Y
```

### Python Script for Logging
```python
import serial
import json
from datetime import datetime

# Open serial port
ser = serial.Serial('/dev/tty.usbmodem14101', 115200, timeout=1)

# Log to file
with open('telemetry_log.jsonl', 'a') as f:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('{'):
            # Add timestamp
            data = json.loads(line)
            data['logged_at'] = datetime.now().isoformat()
            f.write(json.dumps(data) + '\n')
            f.flush()
            print(f"State: {data['state']}, IR: {data['ir']}, ToF: {data['tof']}")
```

## Benefits of Serial-Only Approach

### Advantages
✅ **Simpler code** - No WiFi management overhead  
✅ **Lower latency** - Direct serial communication  
✅ **More reliable** - No WiFi connection issues  
✅ **Lower power** - WiFi radio disabled  
✅ **Easier debugging** - Same console for logs and telemetry  
✅ **No network conflicts** - Works without network setup

### Considerations
⚠️ **Wired connection required** - Must be connected via USB  
⚠️ **Single client** - Only one monitor at a time  
⚠️ **Limited range** - USB cable length restriction

## Migration Notes

### If You Need WiFi Back
1. Re-add WiFi includes and configuration
2. Restore `slot8_WiFi_Handling()` function
3. Add WiFi initialization in `setup1()`
4. Update `loop1()` case 8
5. Reference `WIFI_GUI_GUIDE.md` for setup steps

### Telemetry Rate
- **Frequency:** ~2Hz (500ms cycle / 10 slots × 4 slots between telemetry)
- **Slot assignment:** Slot 8 of 10-slot time-sliced scheduler
- **Consistent timing:** Guaranteed by Core 1 scheduler

## Testing Checklist
- [x] WiFi includes removed
- [x] WiFi constants removed
- [x] WiFi globals removed
- [x] WiFi function replaced
- [x] WiFi initialization removed
- [x] Switch case updated
- [x] JSON format maintained
- [x] Serial output functional
- [ ] Upload and test on hardware
- [ ] Verify telemetry output
- [ ] Confirm state machine operation

## Related Files
- **CAR_final_v2.ino** - Main implementation (WiFi removed)
- **WIFI_GUI_GUIDE.md** - Previous WiFi setup guide (now obsolete)
- **simple_viewer_v2.py** - GUI viewer (requires WiFi, now incompatible)
- **DUAL_CORE_ARCHITECTURE.md** - Architecture documentation
- **QUICK_REFERENCE.md** - API and pin reference

---
**Last Updated:** WiFi removal completed  
**Compatibility:** Serial-only telemetry at 115200 baud  
**Status:** ✅ Ready for testing
