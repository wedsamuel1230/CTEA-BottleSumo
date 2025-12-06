# Feature Specification: TCP Motor Control

**Feature Branch**: `[1-tcp-motor-control]`  
**Created**: 2025-10-25  
**Status**: Draft  
**Input**: User description: "User can control two brushless motors via TCP/IP by setting PWM values (-255 to 255). The car creates a Wi-Fi access point; TCP Server parses commands and maps them to GPIO. Both motors (left/right) must be independently controlled, either together or separately. Commands include start/stop, direction selection, PWM value change. Invalid command inputs trigger error feedback, with no dangerous behavior. Include usability stories, e.g.: user connects, moves car forward, stops, reverses, performs emergency stop. All motor PWM control must use the existing motor.h/motor.cpp library that handles low-level Pico PWM hardware - do not create new PWM implementations."

## Clarifications

### Session 2025-10-25

- Q: Motor driver signaling model?
  → A: Brushless driver with DIR + PWM (driver handles commutation).

- Q: Arduino BSP and networking libraries?
  → A: Arduino-Pico (Earle Philhower) core with WiFi.h and WiFiServer (e.g., `WiFiServer tcp_server(TCP_PORT)`).

- Q: TCP command protocol framing and responses?
  → A: JSON Lines (one JSON object per line) for both commands and responses.

- Q: Authentication approach?
  → A: Session token via AUTH message required before any control commands.

- Q: Edge detection sensors?
  → A: Four QRE1113 reflectance IR sensors are used for border/edge detection, mounted as a 2× front array (front-left, front-right) and 2× rear array (rear-left, rear-right) to protect motion in both directions. Sensors are read via an external ADS1115 I2C ADC (four single‑ended channels A0–A3), connected on Wire1 (SDA=GP26, SCL=GP27). I2C address is configurable within the ADS1115 standard range (0x48–0x4B). Sampling must be non‑blocking and fast enough to meet safety latency targets; thresholds are established via calibration and/or configuration.

## User Scenarios & Testing (mandatory)

### User Story 1 - Connect and Control (Priority: P1)

A coach/operator connects to the robot’s Wi‑Fi access point, establishes a TCP session, and
sends a command to drive forward at moderate speed; the car starts moving; the operator then
stops the car.

Why this priority: This is the primary end-to-end value path (connect → control → stop).

Independent Test: Using a TCP client, connect to the AP, send a valid command to set both
motors to the same positive speed, observe motion start, send stop, observe motion cease.

Acceptance Scenarios:

1. Given the robot is powered on and advertising its AP, When the user connects and opens a
   TCP session, Then the robot acknowledges readiness for commands.
2. Given an established session, When the user sends a valid command to set both motors to
   the same positive value, Then the robot moves forward and returns a success response.
3. Given motion is ongoing, When the user sends a stop command, Then both motors stop and a
   success response is returned.

---

### User Story 2 - Reverse and Direction Control (Priority: P1)

The operator issues a command to reverse direction at a set speed; the car moves backward and
can be stopped reliably.

Why this priority: Direction control is critical to usability and match tactics.

Independent Test: Send a valid command that maps to negative PWM values for both motors,
observe reverse motion, then stop.

Acceptance Scenarios:

1. Given the car is connected and stopped, When the user sends a reverse command/value,
   Then the car moves backward with stable speed, returns success status.
2. Given reverse motion, When the user sends a stop command, Then both motors stop within the
   defined safe response time and a success response is returned.

---

### User Story 3 - Independent Motor Control (Priority: P2)

The operator can command left and right motors independently (e.g., for turning in place or
course corrections).

Why this priority: Independent control enables turning, precision alignment, and obstacle
maneuvering.

Independent Test: Send independent values for left and right motors; observe expected motion
(e.g., differential steering), then stop.

Acceptance Scenarios:

1. Given the car is connected and stopped, When the user sets left=+200 and right=+50,
   Then the robot turns slowly while advancing and returns success status.
2. Given the car is moving differentially, When the user sets left=0 and right=0,
   Then both motors stop and success status is returned.

---

### User Story 4 - Emergency Stop (Priority: P1)

The operator can trigger an emergency stop that immediately ceases all motion regardless of
current state.

Why this priority: Safety is non-negotiable; emergency stop must override all actions.

Independent Test: Send emergency stop during motion; verify immediate cessation of motor
outputs and safe state, with success response.

Acceptance Scenarios:

1. Given forward motion, When the user triggers emergency stop, Then motor outputs drop to
   safe/off immediately and the system reports success.

---

### User Story 5 - Invalid Command Handling (Priority: P1)

Invalid commands or out-of-range values produce clear error feedback and never cause motion
or unsafe behavior.

Why this priority: Prevents accidents and supports reliable operations.

Independent Test: Send malformed or out-of-range commands; verify error responses and no
motion/change to motor state.

Acceptance Scenarios:

1. Given a connected session, When the user sends a nonconforming command, Then the system
   returns an error and no actuation occurs.
2. Given a connected session, When the user sends values outside [-255, 255], Then the system
   saturates inputs safely (or rejects the command per protocol) and returns an error or
   bounded result without unsafe motion.

---

### Edge Cases

- Boundary values: -255, -1, 0, 1, 255 for both motors independently and jointly.
- Out-of-range values: e.g., -999, 999 → saturated or rejected per protocol with no motion
  beyond bounded outputs.
- Rapid direction flips (forward↔reverse) → must remain safe; either ramp or enforce stop
  between opposites per safety rules.
- Session loss (client disconnect) → transition to safe state (motors off) within a bounded
  time.
- Invalid/malformed messages → error feedback with no actuation.
- Malformed JSON or missing required fields → structured JSON error response and no actuation.
- Unauthorized client attempts → commands rejected with error; no actuation.
 - Unauthenticated command before AUTH → error response and no actuation.
 - Invalid token → error response; optionally disconnect after limited retries.
 - Re-AUTH on an authenticated session → either idempotent success or error per protocol; MUST not interrupt safe control state.
- Sensor edge-detection noise or ambient light flicker → require debounced/filtered detection window (e.g., sustained condition ≥ X ms) before triggering safety action.
- Single-sensor failure (stuck high/low) → system should default to safe behavior when in doubt and report sensor health in status.
- I2C errors (NACK, timeouts), missing ADS1115 device, or read saturation → MUST fail safe (motors off) and produce an error/health indicator in telemetry; recovery SHOULD retry without blocking main control.

## Requirements (mandatory)

### Functional Requirements

- FR-001: The system MUST allow a user to connect to the robot’s local wireless network and
  establish a control session over a reliable command channel.
- FR-002: The system MUST accept user commands to set motor speeds for left and right motors
  independently within the integer range [-255, 255]. Value 0 MUST stop the addressed motor.
- FR-003: The system MUST support start/stop, direction (via sign), speed change, and
  emergency stop commands.
- FR-004: The system MUST validate all incoming commands (structure, types, ranges) and MUST
  reject invalid inputs with error feedback; no unsafe actuation may occur for invalid inputs.
- FR-005: On loss of client session or inactivity timeout, the system MUST enter a safe state
  (motors off) within a bounded interval and remain controllable when a valid session resumes.
- FR-006: Only authorized clients may control the robot; unauthorized commands MUST be
  rejected with error feedback.
- FR-007: Command processing MUST ensure both motors can be addressed together or separately
  in a single session without conflict.
- FR-008: The solution MUST produce clear success/error responses for each command, including
  reasons for rejections (e.g., out-of-range, malformed, unauthorized).
- FR-009: Non‑negotiable constraint: All motor PWM control MUST route through the existing
  low‑level motor control module (as defined in project governance); no new PWM implementation
  is permitted in this feature.
- FR-010: On emergency stop, motor actuation MUST cease immediately and remain off until an
  explicit safe command is applied.

- FR-011: Signal model: Each motor uses one DIR pin (direction) and one PWM pin (duty). The
  low-level PWM generation and GPIO control MUST reuse the existing Motor library; no
  servo-style ESC signaling or alternative PWM implementations are permitted.

- FR-012: Protocol framing: Commands and responses MUST use JSON Lines (newline-delimited
  JSON objects). Each command MUST include an "action" field (e.g., "set", "stop",
  "estop") and necessary parameters (e.g., {"action":"set","left":200,"right":200}).
  Each response MUST include {"status":"ok"} or {"status":"error","code":"<ERR>",
  "message":"<details>"}. Unknown actions or missing/invalid fields MUST be rejected with
  an error response and MUST NOT cause actuation.

- FR-013: Authentication: A client MUST send an AUTH command (e.g., {"action":"auth","token":"<secret>"})
  and receive an OK response before any control commands are accepted. Until authenticated,
  only AUTH is permitted; all other actions MUST return error and cause no actuation.
  Repeated failed AUTH attempts MUST trigger rate limiting or disconnect. Tokens MUST NOT be
  logged in plaintext in normal operation.

- FR-014: Edge detection: The system MUST integrate four QRE1113 reflectance IR sensors for edge/border detection. Each sensor MUST be sampled periodically and processed in a non-blocking manner with basic debouncing/filtering. The design MUST support four channels using available GPIO (analog ADC or digital RC-time) without introducing unsafe latency to motor control.

- FR-015: Safety override: When an edge is detected in the current intended direction of motion, the system MUST immediately transition to a safe state (motors off) and reject further motion commands that would continue toward the detected edge until the condition clears (sensor(s) return to safe state for a short, configurable dwell time). Emergency stop takes precedence over all states.

#### Safety Pattern Mapping (Retreat Behavior)

For clarity and predictable behavior, the following default retreat vectors are applied when specific sensor patterns are detected. Channel-to-position mapping: A0 = front-left, A1 = front-right, A2 = rear-left, A3 = rear-right. Sensor flags form a 4-bit pattern [A3 A2 A1 A0]. Motor speed values below are duty percentages [-100..100]; implementation maps these to the existing Motor library outputs.

| Pattern (Binary) | Hex | Sensors Detected         | Action                    | Motor Speeds (L, R)
|------------------|-----|--------------------------|---------------------------|---------------------|
| `0b0001`         | 0x1 | A0 only                  | RETREAT_BACKWARD_RIGHT    | (-80, -40)         |
| `0b0010`         | 0x2 | A1 only                  | RETREAT_BACKWARD_LEFT     | (-40, -80)         |
| `0b0100`         | 0x4 | A2 only                  | RETREAT_FORWARD_RIGHT     | (80, 40)           |
| `0b1000`         | 0x8 | A3 only                  | RETREAT_FORWARD_LEFT      | (40, 80)           |
| `0b0011`         | 0x3 | A0+A1 (front/top)        | RETREAT_BACKWARD          | (-80, -80)         |
| `0b1100`         | 0xC | A2+A3 (rear/bottom)      | RETREAT_FORWARD           | (80, 80)           |
| `0b0101`         | 0x5 | A0+A2 (left side)        | RETREAT_RIGHT             | (60, -60)          |
| `0b1010`         | 0xA | A1+A3 (right side)       | RETREAT_LEFT              | (-60, 60)          |
| `0b1001`         | 0x9 | A0+A3 (diagonal)         | RETREAT_FORWARD_RIGHT     | (80, 40)           |
| `0b0110`         | 0x6 | A1+A2 (diagonal)         | RETREAT_FORWARD_LEFT      | (40, 80)           |

Emergency Stop rule: Any pattern with three or more sensors active (≥3 bits set) MUST trigger EMERGENCY_STOP (immediate motors off) instead of a retreat vector.

- FR-016: Calibration: The system MUST provide a way to establish or update sensor thresholds (e.g., a "calibrate" command or a documented boot-time calibration flow). Calibration MUST be feasible on typical bottle-sumo mats (white border, dark interior) and MUST not require recompilation.

- FR-017: Telemetry: On request (e.g., a "status" action), the system MUST return current sensor readings/states alongside authorization and motion state so operators can verify edge-detection behavior.

- FR-018: Sensor interface (ADS1115): QRE1113 sensors MUST be read through an ADS1115 I2C ADC using four single‑ended channels. The system MUST handle I2C initialization, address selection, and per‑read errors robustly. Read failures MUST NOT block the main loop and MUST default to a safe state.

### Non-Functional Requirements

- NFR-001: Preferred Arduino environment: Arduino-Pico (Earle Philhower) core. Networking via
  WiFi.h and WiFiServer for the TCP server. This choice informs examples and development
  workflow but does not alter functional behavior or protocol.

- NFR-002: Authentication token MUST be at least 12 characters. Storage and handling MUST
  avoid accidental disclosure in logs; debug builds MAY redact or hash tokens when necessary
  for diagnostics.

- NFR-003: Sensor sampling and processing MUST be non-blocking (no long delays in the main loop) and robust to ambient light variation. A minimal filtering window (e.g., 5–20 ms) SHOULD be used to reduce false positives without delaying legitimate edge reactions beyond safe stopping distance.

- NFR-004: I2C behavior: The ADS1115 data rate and bus configuration MUST support effective edge detection across all four channels with total detection latency (sensor change to safe-state action) under a target threshold (see SC-007) while avoiding starvation of networking and command processing tasks.

### Key Entities

- Command: Represents a request from the operator to control motion; includes target (left,
  right, both), value(s) in [-255, 255], and action type (set, stop, emergency stop).
- Feedback: The system’s response to a command with status (success/error) and optional
  message/code for diagnostics.
- Session: Represents a logical control connection between the operator and the robot.

## Success Criteria (mandatory)

### Measurable Outcomes

- SC-001: A new operator can connect and successfully perform forward → stop within one
  minute, following concise instructions.
- SC-002: 100% of boundary tests for PWM values {-255, -1, 0, 1, 255} result in correct,
  safe behavior for left and right motors independently and jointly.
- SC-003: 100% of out-of-range tests (e.g., -999, 999) result in safe handling (saturation or
  rejection) with no unintended motion.
- SC-004: 100% of invalid/malformed command tests produce error feedback and no actuation.
- SC-005: Emergency stop halts actuation immediately in 100% of tests and remains in safe
  state until a valid resume command is applied.
- SC-006: Unauthorized control attempts are rejected in 100% of tests.

- SC-007: From a clear edge crossing under typical lighting, the system detects the edge and commands motors to safe/off within ≤ 50 ms in 95% of trials and ≤ 100 ms in 100% of trials at moderate approach speeds.
