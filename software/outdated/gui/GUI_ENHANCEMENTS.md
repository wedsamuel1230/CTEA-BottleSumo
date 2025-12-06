# GUI Enhancement Summary

## Changes Made to viewer.py

### 1. ✅ Added Voltage Progress Bars

**What was added:**
- Progress bars next to each voltage reading in the Sensors section
- Visual representation of voltage levels (0-3.3V range)
- Bars automatically update as voltage values change

**Technical details:**
- Added `sensor_voltage_bars` list to store `ttk.Progressbar` widgets
- Set maximum value to 3.3V (typical ESP32 ADC range)
- Progress bars are 150 pixels wide and expand with the window
- Values are clamped between 0.0 and 3.3V for safety

**Location in code:**
- Lines ~168-191: Created progress bar widgets in UI
- Lines ~307-311: Update progress bar values in `_update_display()`

### 2. ✅ Made Window Draggable

**What was added:**
- Click and drag anywhere on the window to move it
- Smooth dragging experience
- No need to rely only on the title bar

**Technical details:**
- Added `_drag_start_x` and `_drag_start_y` instance variables
- Created `_setup_window_drag()` method to bind mouse events
- `_start_drag()` records initial click position
- `_on_drag()` calculates and applies window position changes
- Binds to `<Button-1>` (mouse down) and `<B1-Motion>` (mouse drag)

**Location in code:**
- Lines ~131-134: Initialize drag variables in `__init__`
- Lines ~246-262: Drag handler methods

### 3. 🔧 Window Size Adjustment

**Why changed:**
- Increased window width from 520px to 650px to accommodate progress bars
- Minimum width increased from 500px to 600px

## How to Use

### Voltage Bars
- Bars fill from left to right as voltage increases
- Full bar = 3.3V
- Empty bar = 0V
- Updates in real-time with sensor data

### Draggable Window
- Click anywhere on the window background
- Hold mouse button and drag to move the window
- Release to drop in new position

## Testing the Changes

Run the viewer:
```bash
cd realtime_monitor
source .venv/bin/activate
python viewer.py
```

Or use the convenience script:
```bash
./run_viewer.sh
```

## Visual Layout

```
┌─────────────────────────────────────────────────┐
│  Bottle Sumo Real-Time Viewer                   │
├─────────────────────────────────────────────────┤
│  Connection: [Host] [Port] [Connect]            │
├─────────────────────────────────────────────────┤
│  Sensors:                                        │
│  Index   Raw    Voltage(V)   Level              │
│  Sensor 0  1234   2.456    [████████░░]         │
│  Sensor 1  5678   1.234    [████░░░░░░]         │
│  Sensor 2  ...    ...      [██████████]         │
│  Sensor 3  ...    ...      [░░░░░░░░░░]         │
├─────────────────────────────────────────────────┤
│  Robot State: ...                                │
│  System Info: ...                                │
└─────────────────────────────────────────────────┘
```

## Notes

- Progress bars use native ttk styling (adapts to OS theme)
- Drag functionality works with all mouse buttons bound to Button-1
- Window position is preserved during drag but not saved between sessions
