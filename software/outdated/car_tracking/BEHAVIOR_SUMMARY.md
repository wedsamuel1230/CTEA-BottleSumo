# Car Tracking Behavior - Point and Stop Mode

## Current Behavior (Updated 2025-11-10)

The car now **rotates in place** to face detected objects and **stops when centered**. It does NOT move forward.

## Serial Monitor Output Format

```
[CORE0] Direction: ✓ CENTERED      | M0= 250mm bias=+0.05 L=+0.0 R=+0.0 | R45: 300 R23: 280 M0: 250 L23: 290 L45: 310
[CORE0] Direction: ← TURN LEFT     | M0= 150mm bias=+0.45 L=-30.0 R=+30.0 | R45: 400 R23: 380 M0: 150 L23: 120 L45: 100
[CORE0] Direction: TURN RIGHT →    | M0= 200mm bias=-0.38 L=+30.0 R=-30.0 | R45: 100 R23: 120 M0: 200 L23: 350 L45: 380
```

## Direction Indicators

- **`✓ CENTERED`** - Object is directly in front, car is stopped
- **`← TURN LEFT`** - Object detected more on left side, car rotating left (counter-clockwise)
- **`TURN RIGHT →`** - Object detected more on right side, car rotating right (clockwise)
- **`NO OBJECT (stopped)`** - No valid sensor readings, car stopped
- **`SEARCHING...`** - Recently lost object, waiting before stopping

## Motor Behavior

### Rotation Left (Object on Left)
- Left Motor: **-30.0** (reverse)
- Right Motor: **+30.0** (forward)
- Result: Car spins counter-clockwise in place

### Rotation Right (Object on Right)
- Left Motor: **+30.0** (forward)
- Right Motor: **-30.0** (reverse)
- Result: Car spins clockwise in place

### Centered (Object in Front)
- Left Motor: **0.0** (stopped)
- Right Motor: **0.0** (stopped)
- Result: Car is stationary, facing object

## Key Parameters

```cpp
const float ROTATION_SPEED = 30.0f;   // Speed for rotating (0-100)
const float BIAS_DEADZONE = 0.15f;    // Threshold for "centered" (-1.0 to +1.0)
const uint16_t DETECT_MIN_MM = 50;    // Minimum detection distance
const uint16_t DETECT_MAX_MM = 1000;  // Maximum detection distance
```

## Tuning Guide

### Make rotation faster/slower
```cpp
const float ROTATION_SPEED = 50.0f;   // Increase = faster rotation
```

### Make "centered" detection more/less strict
```cpp
const float BIAS_DEADZONE = 0.05f;    // Smaller = must be more precisely centered
const float BIAS_DEADZONE = 0.25f;    // Larger = wider "centered" range
```

### Change detection range
```cpp
const uint16_t DETECT_MAX_MM = 500;   // Shorter = only close objects
const uint16_t DETECT_MAX_MM = 2000;  // Longer = far objects too
```

## Testing Checklist

1. ✅ Upload code to Pico W
2. ✅ Open Serial Monitor (115200 baud)
3. ✅ Place object in front - should show "✓ CENTERED"
4. ✅ Move object to left - should show "← TURN LEFT" and car rotates left
5. ✅ Move object to right - should show "TURN RIGHT →" and car rotates right
6. ✅ Remove object - should show "NO OBJECT (stopped)" after 2 seconds
7. ✅ Check motor directions match serial output (L= and R= values)

## Expected Serial Output Example

```
[CORE0] Direction: ← TURN LEFT     | M0= 180mm bias=+0.32 L=-30.0 R=+30.0 | R45: 450 R23: 400 M0: 180 L23: 150 L45: 120
[CORE0] Direction: ← TURN LEFT     | M0= 190mm bias=+0.28 L=-30.0 R=+30.0 | R45: 420 R23: 380 M0: 190 L23: 165 L45: 140
[CORE0] Direction: ← TURN LEFT     | M0= 195mm bias=+0.18 L=-30.0 R=+30.0 | R45: 400 R23: 370 M0: 195 L23: 180 L45: 160
[CORE0] Direction: ✓ CENTERED      | M0= 200mm bias=+0.08 L=+0.0 R=+0.0 | R45: 380 R23: 360 M0: 200 L23: 190 L45: 175
```

## Differences from Previous Version

| Feature | Old Behavior | New Behavior |
|---------|-------------|--------------|
| Forward Motion | Moved at MAX_SPEED when clear | **Never moves forward** |
| Rotation | Differential steering while moving | **Pure rotation in place** |
| Serial Output | "Track: dist=..." | **"Direction: ..."** with symbols |
| Centered State | Still moves forward | **Stops completely** |
| Motor Display | Only positive speeds | **Shows +/- for direction** |

## Why This Design?

- **Safety:** Car won't chase/collide with objects
- **Pointing:** Always faces detected object
- **Simple:** Easy to verify car is tracking correctly
- **Debug-friendly:** Clear direction indication in serial output
- **Next Step Ready:** Easy to add forward motion later if needed
