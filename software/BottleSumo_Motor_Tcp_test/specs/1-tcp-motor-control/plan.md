# Implementation Plan: TCP Motor Control

**Branch**: `[1-tcp-motor-control]` | **Date**: 2025-10-25 | **Spec**: specs/1-tcp-motor-control/spec.md
**Input**: Feature specification from `/specs/1-tcp-motor-control/spec.md`

## Summary

Implement a Pico W firmware that exposes a Wi‑Fi AP TCP server using JSON Lines commands to control two brushless motors via existing `Motor.h/.cpp` (DIR + PWM) with safety features: AUTH gating and four QRE1113 edge sensors read via ADS1115 I2C. No dedicated hardware E‑STOP/FAULT inputs are used; safety relies on software estop command and sensor gating. The main loop is non‑blocking, with fast sensor sampling and immediate safety override (≤50 ms typical) to cut motor outputs. All PWM goes through the existing Motor library; no new PWM implementations.

## Technical Context

**Language/Version**: Arduino C++ (RP2040, Earle Philhower core)
**Primary Dependencies**: Arduino-Pico (WiFi.h, WiFiServer); ArduinoJson; ADS1115 driver (e.g., Adafruit ADS1X15 or compatible); existing `Motor.h/.cpp`
**Storage**: N/A
**Testing**: AUnit (or serial-based assertions) + host-side log verification; manual motion tests on stand
**Target Platform**: Raspberry Pi Pico W (RP2040 + CYW43 Wi‑Fi)
**Project Type**: Single embedded firmware (Arduino sketch + library)
**Performance Goals**: Edge detection to safe-off: ≤50 ms in 95% trials; ≤100 ms worst-case
**Constraints**: Non‑blocking loop; resilient Wi‑Fi; <64 KB heap usage; robust to ambient light changes
**Scale/Scope**: Single robot; single TCP client session

## Constitution Check

- P1 GPIO Discipline: Define GPIO map (2×PWM, 2×DIR) and I2C pins. Adjustable via constants, documented in README. PASS
- P2 Network Role: Pico W AP-mode TCP server; single client; inactivity timeout → safe state. PASS
- P3 Security: AUTH token required before control; allowlist actions; rate limit failed AUTH; JSON validation/saturation. PASS
- P4 Electrical Safety: Software estop command immediate cut; sensor gating prevents unsafe motion; note series resistors for GPIO; verify driver logic levels. PASS
- P5 Maintainability: Modules: wifi_server, command_parser, safety_manager, sensors_ads1115, motors (wrapper around Motor lib); comments + quickstart; formatting optional. PASS
- P6 Command Safety: Validate ranges; saturate; reject malformed/unknown; safe on session loss/I2C errors. PASS
- P7 PWM Edge Cases: Tests for {-255, -1, 0, 1, 255, -999, 999}; verify saturation and no unsafe motion. PASS
- P8 Open Source: Ensure LICENSE present; semantic commits; PR reviews. FOLLOW-UP if missing
- P9 Library Reuse: All PWM flows through `Motor` class API. PASS

## Project Structure

### Documentation (this feature)

```text
specs/1-tcp-motor-control/
├── plan.md
├── research.md
├── data-model.md
├── quickstart.md
├── contracts/
└── tasks.md  (created later by /speckit.tasks)
```

### Source Code (repository root)

```text
src/
├── main.ino                  # setup/loop, scheduler, safety glue
├── wifi_server.cpp/.h        # AP setup, WiFiServer accept, client IO
├── command_parser.cpp/.h     # JSONL parsing (ArduinoJson), command routing
├── safety_manager.cpp/.h     # Sensor gating + software estop command logic
├── sensors_ads1115.cpp/.h    # ADS1115 init, read, debounce/thresholding
├── motors.cpp/.h             # Wrapper to use existing Motor class for L/R
└── telemetry.cpp/.h          # status payloads (sensors/auth/motion)

lib/
└── Motor/                    # existing Motor.h/.cpp (or reference current paths)

tests/
└── unit/                     # AUnit or serial-driven checks
```

**Structure Decision**: Single embedded firmware with cohesive modules to keep loop non‑blocking and testable, while reusing `Motor.h/.cpp` via a thin wrapper and centralizing safety.

## Hardware Design

- MCU: Raspberry Pi Pico W
- Drivers: Dual brushless drivers with DIR + PWM per motor (e.g., DRV series)
- Sensors: 4× QRE1113 reflectance sensors via ADS1115 (A0–A3)
- Inputs: None dedicated for E‑STOP/FAULT (handled via software command)
- I2C: ADS1115 on Wire1 (SDA GP26, SCL GP27; address 0x48–0x4B configurable)

### GPIO Map (proposed, adjustable)

- Left Motor PWM: GP11 (PWM)
- Left Motor DIR:  GP10
- Right Motor PWM: GP14 (PWM)
- Right Motor DIR: GP13
- I2C1 (Wire1) SDA: GP26
- I2C1 (Wire1) SCL: GP27

Notes:
- Most RP2040 pins support PWM; the specific slices will be chosen by the Motor lib `begin()`.
- Use inline series resistors (1–4.7 kΩ) on signal lines as appropriate and ensure correct logic polarity to driver inputs.

## Software Architecture

- Main loop co-operative scheduler using millis():
  - Tasks: wifi_io, parse_commands, sample_sensors, safety_eval, apply_motors, telemetry
- Modules:
  - wifi_server: Configure AP, WiFiServer listen, read lines non‑blocking
  - command_parser: ArduinoJson to parse JSON per line; enforce schema and AUTH gate
  - sensors_ads1115: Initialize ADS1115 (Wire1 GP26/27), set data rate; read channels; debounce; classify edge
  - safety_manager: Combine sensor edge flags and software estop; if unsafe → force motors off + latch until clear
  - motors: Wrap existing `Motor` for left/right; map [-255,255] to duty [-100,100]
  - telemetry: Build status responses (sensor states, auth state, motion state)

## Communication Protocol (JSON Lines over TCP)

- One command per line; responses echo status per FR-012
- Actions:
  - auth: {"action":"auth","token":"…"}
  - set: {"action":"set","left":int,-255..255,"right":int,-255..255}
  - stop: {"action":"stop"}
  - estop: {"action":"estop"}
  - status: {"action":"status"}
  - calibrate: {"action":"calibrate","mode":"auto|manual","samples":opt}
- Error: {"status":"error","code":"…","message":"…"}

## Implementation Steps

1) Scaffolding
- Add src/ and lib/Motor (or update include paths to reuse existing Motor files)
- Create module headers/sources with clear APIs

2) Motor integration
- Include Motor.h/.cpp; create two instances (left/right)
- Map value [-255,255] to duty [-100,100], sign → DIR; 0 → stop()

3) Wi‑Fi AP and TCP server
- Configure AP SSID/PSK; start WiFiServer(PORT)
- Accept single client; implement readLine with timeout; heartbeat timeout triggers safe-off

4) Command parser
- Use ArduinoJson; strict schema; range checks; AUTH gate; rate limit failed auth

5) ADS1115 sensors
- Init Wire1 (GP26/GP27) and ADS1115 address; pick data rate (~860 SPS)
- Periodic read 4 channels; debounce 5–20 ms; threshold via calibration values

6) Safety manager
- Inputs: Software estop command and edge flags; if unsafe → force stop and latch until clear dwell
- Block set commands that would drive toward detected edge

7) Telemetry and calibration
- status: include sensor raw/flags, auth, motion
- calibrate: record baseline for white/black and compute thresholds

8) Tests
- Unit tests for parser/range/safety transitions (AUnit or serial harness)
- Bench test for SC-007 latency with timestamp logs

## Test Plan (Automated + Manual)

- Unit: parser accepts valid, rejects malformed/out-of-range; mapping saturation; estop dominance
- Integration: simulate sensor flags → motors forced off; session loss timeout → off
- Latency: log t(sensor change) to t(motors=off) ≤50 ms typical (≤100 ms worst)
- Wi‑Fi: disconnect during motion → safe-off

## Risk Management

- Wi‑Fi reliability: implement inactivity timeout and safe defaults
- GPIO protection: series resistors as needed; verify logic polarity with driver inputs
- Command failure: reject with clear error; never actuate on invalid/unauthenticated
- I2C errors: treat as unsafe; retry in background without blocking

## Dependencies & Inclusion

- Reuse existing `Motor.h`/`Motor.cpp` from repository; include via lib/Motor or update include paths
- ArduinoJson for JSONL parsing (lightweight configuration)
- ADS1115 library compatible with Arduino-Pico

