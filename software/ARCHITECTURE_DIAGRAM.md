# BottleSumo Time-Sliced Architecture
# Motor & Test Mode Integration

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                  Raspberry Pi Pico W (RP2040)                   │
├────────────────────────────┬────────────────────────────────────┤
│         CORE 0             │           CORE 1                   │
│    State Machine           │        I/O Hub                     │
│    (~100Hz)                │      (~2.7Hz cycle)                │
├────────────────────────────┼────────────────────────────────────┤
│                            │                                    │
│  ┌──────────────────┐      │  ┌──────────────────────────┐     │
│  │ Read Sensor Data │      │  │ TASK 1: ADS Read (10ms)  │     │
│  │ from Shared Mem  │      │  └──────────────────────────┘     │
│  └──────────────────┘      │              ↓                     │
│           ↓                │  ┌──────────────────────────┐     │
│  ┌──────────────────┐      │  │ TASK 2: ToF Read (250ms) │     │
│  │ Decision Logic   │      │  │   • 5 sensors @ 50ms     │     │
│  │ • Edge detection │      │  │   • Long range stable    │     │
│  │ • Opponent detect│      │  └──────────────────────────┘     │
│  │ • Action select  │      │              ↓                     │
│  └──────────────────┘      │  ┌──────────────────────────┐     │
│           ↓                │  │ TASK 3: ADS Read (10ms)  │     │
│  ┌──────────────────┐      │  └──────────────────────────┘     │
│  │ Generate Motor   │      │              ↓                     │
│  │ Commands         │      │  ┌──────────────────────────┐     │
│  │ (-100 to +100)   │      │  │ TASK 4: Button (5ms)     │     │
│  └──────────────────┘      │  │ TASK 5: Debounce (20ms)  │     │
│           ↓                │  └──────────────────────────┘     │
│  ┌──────────────────┐      │              ↓                     │
│  │ Write to Shared  │      │  ┌──────────────────────────┐     │
│  │ Motor Commands   │      │  │ TASK 6: ADS Read (10ms)  │     │
│  └──────────────────┘      │  └──────────────────────────┘     │
│           ↓                │              ↓                     │
│       delay(10ms)          │  ┌──────────────────────────┐     │
│           ↑                │  │ TASK 7: Motor PWM (5ms)  │◄────┼─── Priority:
│           └────────────────┤  │ • Check emergency stop   │     │    1. E-Stop
│                            │  │ • Test mode OR normal    │     │    2. Test Mode
│                            │  │ • Apply PWM to motors    │     │    3. Normal
│                            │  └──────────────────────────┘     │
│                            │              ↓                     │
│                            │  ┌──────────────────────────┐     │
│                            │  │ TASK 8: WiFi/TCP (50ms)  │     │
│                            │  │ • Accept clients         │     │
│                            │  │ • Handle commands        │     │
│                            │  │ • Stream telemetry       │     │
│                            │  └──────────────────────────┘     │
│                            │              ↓                     │
│                            │  ┌──────────────────────────┐     │
│                            │  │ TASK 9: ADS Read (10ms)  │     │
│                            │  └──────────────────────────┘     │
│                            │              ↓                     │
│                            │       Cycle complete               │
│                            │    (Total: ~370ms, ~2.7Hz)         │
└────────────────────────────┴────────────────────────────────────┘
                                        │
                                        │ WiFi AP
                                        │ TCP Port 4242
                                        ↓
                      ┌─────────────────────────────┐
                      │   Python GUI (viewer.py)    │
                      ├─────────────────────────────┤
                      │ ┌─────────────────────────┐ │
                      │ │  Connection Controls    │ │
                      │ └─────────────────────────┘ │
                      │ ┌─────────────────────────┐ │
                      │ │  🆕 Test Mode Section   │ │
                      │ │  • Mode selector        │ │
                      │ │  • Apply button         │ │
                      │ │  • Current mode display │ │
                      │ └─────────────────────────┘ │
                      │ ┌─────────────────────────┐ │
                      │ │  IR Sensors (4x)        │ │
                      │ │  • Raw / Voltage        │ │
                      │ │  • Visual bars          │ │
                      │ └─────────────────────────┘ │
                      │ ┌─────────────────────────┐ │
                      │ │  Motor Control          │ │
                      │ │  • Slider -100 to +100  │ │
                      │ │  • Enable checkbox      │ │
                      │ │  • Status indicator     │ │
                      │ └─────────────────────────┘ │
                      │ ┌─────────────────────────┐ │
                      │ │  Threshold Config       │ │
                      │ │  • Per-sensor adjust    │ │
                      │ └─────────────────────────┘ │
                      │ ┌─────────────────────────┐ │
                      │ │  ToF Sensors (3x)       │ │
                      │ │  • Distance display     │ │
                      │ │  • Proximity bars       │ │
                      │ └─────────────────────────┘ │
                      │ ┌─────────────────────────┐ │
                      │ │  Robot State & System   │ │
                      │ └─────────────────────────┘ │
                      └─────────────────────────────┘
```

## Test Modes

```
┌─────────────────────────────────────────────────────────────────┐
│                         Test Modes                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  AUTO              Normal autonomous operation                  │
│                    Core 0 controls motors                       │
│                                                                 │
│  TEST_MOTOR        Direct motor PWM control                     │
│                    GUI sliders → TEST_MOTOR cmd → Motors       │
│                    5-second safety timeout                      │
│                                                                 │
│  TEST_SENSOR       Individual sensor testing                    │
│                    Focus on specific IR/ToF sensor             │
│                    High-frequency raw data stream              │
│                                                                 │
│  CALIBRATE_IR      IR sensor calibration                        │
│                    Record min/max over white/black             │
│                    Suggest optimal thresholds                  │
│                                                                 │
│  CALIBRATE_TOF     ToF sensor calibration                       │
│                    Record distance ranges                       │
│                    Validate sensor accuracy                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Data Flow

```
Sensors → Core 1 → Shared Memory → Core 0 → Decision Logic
                                              ↓
Motor Commands ← Core 1 ← Shared Memory ← Motor Command
   (PWM)              ↑
                Test Mode Override
                      ↑
              GUI via TCP Commands
```

## TCP Command Protocol

```
┌─────────────────────────────────────────────────────────────────┐
│                     TCP Commands (Port 4242)                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  SET_MODE <mode>                                                │
│    → Changes test mode (AUTO/TEST_MOTOR/TEST_SENSOR/...)       │
│    ← {"ack":"mode_set","mode":"<mode>"}                        │
│                                                                 │
│  GET_MODE                                                       │
│    ← {"mode":"<mode>","uptime_sec":<seconds>}                  │
│                                                                 │
│  TEST_MOTOR <left> <right>                                      │
│    → Sets motor PWM (-100 to +100)                             │
│    ← "OK: Motors set to LEFT=<left> RIGHT=<right>"            │
│                                                                 │
│  STOP_MOTOR                                                     │
│    → Emergency stop                                             │
│    ← "OK: Emergency stop activated"                            │
│                                                                 │
│  GET_MOTOR                                                      │
│    ← Motor status (PWM, E-Stop, timeout)                       │
│                                                                 │
│  threshold,<idx>,<value>                                        │
│    → Sets IR threshold (0.1-4.0V)                              │
│    ← {"ack":"threshold_set","sensor":<idx>,"value":<val>}     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Telemetry Stream (JSON)

```json
{
  "schema": "2.0",
  "ts": 12345,
  "ir": {
    "raw": [1234, 2345, 3456, 4567],
    "volts": [1.234, 2.345, 3.456, 0.123]
  },
  "tof": {
    "dist": [500, 1200, 800, 1500, 300],
    "valid": [true, true, true, true, false]
  },
  "mode": "RUN",
  "test_mode": "AUTO",
  "motors": {
    "left": 0,
    "right": 0,
    "estop": false,
    "timeout_sec": 0
  }
}
```

## Pin Assignments

```
┌─────────────────────────────────────────────────────────────────┐
│                      Hardware Connections                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Motors:                                                        │
│    Left Motor PWM  → GP6                                        │
│    Left Motor DIR  → GP7                                        │
│    Right Motor PWM → GP8                                        │
│    Right Motor DIR → GP9                                        │
│                                                                 │
│  ToF Sensors (Wire1, SDA=GP26, SCL=GP27):                      │
│    XSHUT Pins → GP12, GP11, GP13, GP10, GP14                   │
│    I2C Addr   → 0x30, 0x31, 0x32, 0x33, 0x34                   │
│                                                                 │
│  IR Sensors (Wire, default pins):                              │
│    ADS1115 → 0x48                                               │
│                                                                 │
│  Buttons:                                                       │
│    Test Mode → GP15                                             │
│    Run Mode  → GP16                                             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

*Architecture Diagram - Motor & Test Mode Integration*
*Date: October 16, 2025*
