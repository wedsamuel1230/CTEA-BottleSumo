# Research Journal: TCP Motor Control

Date: 2025-10-25
Feature: specs/1-tcp-motor-control/spec.md

## Decisions and Rationale

### D1. Sensor Interface: ADS1115 I2C for 4× QRE1113
- Decision: Use ADS1115 (A0–A3) to read all four QRE1113 sensors via Wire1 (I2C1) on GP26/GP27 (SDA/SCL), addr 0x48–0x4B configurable.
- Rationale: Provides four high-resolution ADC channels without consuming RP2040 ADC; stable sampling at high data rates; frees GPIO flexibility.
- Alternatives: RP2040 ADC (limited channels/precision), RC‑time on GPIO (timing sensitive, more CPU), external comparator (additional hardware).

### D2. Protocol: JSON Lines over TCP
- Decision: Single client TCP with JSONL framing; commands include auth, set, stop, estop, status, calibrate.
- Rationale: Human‑readable, line‑delimited, easy to test; ArduinoJson simplifies validation.
- Alternatives: Binary framing (faster but harder to test), HTTP over Wi‑Fi (heavier), UDP (unreliable for control).

### D3. Security: Session AUTH token (≥12 chars)
- Decision: Require AUTH before control; rate‑limit failed attempts; avoid plaintext tokens in logs.
- Rationale: Reduces accidental/abusive control; aligns with spec NFR.
- Alternatives: PSK+TLS (heavy on Pico W), no auth (unsafe).

### D4. Safety Inputs: Software only (no dedicated hardware inputs)
- Decision: Do not allocate hardware E‑STOP/FAULT inputs. Safety provided by software estop command and sensor‑based edge gating.
- Rationale: Simplifies wiring and frees GPIO; meets safety needs via immediate software cut‑off and robust sensor handling.
- Alternatives: Hardware E‑STOP and shared FAULT inputs (more pins, added wiring) if future risk analysis demands it.

### D5. GPIO Map
- Decision: PWM L: GP11, DIR L: GP10; PWM R: GP14, DIR R: GP13; I2C1 (Wire1): GP26 (SDA), GP27 (SCL).
- Rationale: Aligns with existing sketch and keeps pins contiguous for motor control; Wire1 matches prior hardware.
- Alternatives: Other GP pins if Motor library PWM slice conflicts; I2C0 mapping if Wire1 unavailable.

### D6. Non‑blocking Architecture
- Decision: Cooperative scheduler via millis(); ADS1115 reads staggered; no delays in loop; timeouts enforced.
- Rationale: Ensures responsive Wi‑Fi, parsing, and safety reaction times.
- Alternatives: delay()-based timing (blocks); RTOS (overkill for scope).

## Patterns and Best Practices

- ArduinoJson: Use StaticJsonDocument with bounded sizes; validate types and ranges strictly.
- ADS1115: Use highest suitable data rate (e.g., 860 SPS); single‑ended mode; oversampling optional; handle NACK/timeouts gracefully.
- Safety: Debounce sensors (5–20 ms); treat I2C errors as unsafe; estop dominates all states; require dwell to clear.
- Motor control: Map [-255,255] → duty [-100,100]; 0 maps to stop(); sign selects DIR; all via Motor API.
- Networking: Inactivity timeout; safe‑off on disconnect; limit command rate if needed.

## Open Items Resolved

- Sensor variant: ADS1115 confirmed.
- Protocol framing: JSON Lines confirmed.
- Auth approach: Session token confirmed.
- Safety approach: Software estop + sensor gating (no dedicated FAULT input).

## References

- Raspberry Pi Pico W pinout (PWM/I2C capabilities)
- Adafruit ADS1X15 Arduino library (for ADS1115)
- ArduinoJson documentation
