---

description: "Tasks for 1-tcp-motor-control feature implementation"
---

# Tasks: 1-tcp-motor-control

**Input**: Design documents from `/specs/1-tcp-motor-control/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Only include tests where explicitly noted in spec/plan or helpful for smoke validation. We’ll favor serial-based checks for embedded.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Single embedded project layout per plan.md:
  - `src/`, `lib/`, `tests/`, `docs/` at repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create folders, docs, and baseline files per implementation plan

- [ ] T001 Create project structure per implementation plan: `src/`, `lib/`, `tests/`, `docs/`
- [ ] T002 Create docs: `docs/motor_api.md` (document API of existing `Motor.h/.cpp`)
- [ ] T003 Create docs: `docs/gpio_wiring.md` (GPIO map, ADS1115 Wire1 GP26/GP27, motor pins GP10/11/13/14)
- [ ] T004 Create README skeleton with Arduino Pico W upload/build steps: `README.md`
- [ ] T005 Place or reference existing Motor library under `lib/Motor/` and verify includes resolve (no file moves if already present)
 - [ ] T053 Decide include method (library path `lib/Motor/` with `#include <Motor/Motor.h>` vs sketch-relative) and document in `docs/motor_api.md`; update `src/motor_controller.h` includes accordingly
 - [ ] T054 Perform Arduino build sanity check to confirm Motor includes resolve (Arduino IDE or CLI): result notes in `README.md`
 - [ ] T055 Add LICENSE (MIT) at repo root to satisfy Constitution P8; reference in README

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Minimal scaffolding that all stories build upon (non-blocking loop, command routing, error response shape)

- [ ] T006 Create sketch entrypoint with non-blocking loop skeleton: `src/main.ino` (setup/loop, millis scheduler placeholders)
- [ ] T007 [P] Define command/response constants and error codes per contracts: `src/command_parser.h`
- [ ] T008 [P] Create command parser interface and stub: `src/command_parser.cpp` + `src/command_parser.h`
- [ ] T009 [P] Create motor controller wrapper interface and stub (uses Motor.h/.cpp): `src/motor_controller.cpp` + `src/motor_controller.h`
- [ ] T010 [P] Add minimal serial diagnostics (baud setup, log helpers): `src/main.ino`
- [ ] T011 Configure WiFi AP constants (SSID/PSK/PORT) and single-client policy placeholders: `src/main.ino`
- [ ] T012 Wire include paths to use `lib/Motor/Motor.h` from wrappers: `src/motor_controller.h`
 - [ ] T043 Implement inactivity/disconnect safe-off with measurable timeout (e.g., 1500 ms): `src/main.ino`
   Acceptance: From last valid command receipt or socket activity, if timeout elapses or client disconnects, both motors are forced off within 100 ms and further motion is gated until a new session/auth cycle.

**Checkpoint**: Foundation ready - can accept TCP, parse commands, and drive motors per stories incrementally

---

## Phase 3: User Story 1 - Connect and Control (Priority: P1) 🎯 MVP

**Goal**: Operator connects to AP, authenticates, sets both motors forward at moderate speed, then stops

**Independent Test**: Connect via TCP, send AUTH, send `{"action":"set","left":200,"right":200}`, verify motion, then `{"action":"stop"}` and verify stop

### Implementation for User Story 1

- [ ] T013 Implement WiFi AP + TCP server accept/readline (single client): `src/main.ino`
- [ ] T014 Implement AUTH command (session token gating, ok/error replies): `src/command_parser.cpp`
- [ ] T015 Map set/stop actions to wrapper calls; return JSONL responses: `src/command_parser.cpp`
- [ ] T016 Implement value mapping [-255,255] → duty [-100,100] (0→stop, sign→DIR): `src/motor_controller.cpp`
- [ ] T017 Initialize two Motor instances (L/R) and bind to pins (GP11/GP10 and GP14/GP13): `src/motor_controller.cpp`
- [ ] T018 Add serial logs: connection, auth status, set/stop summaries: `src/main.ino`
- [ ] T019 Document quick usage in README (connect/auth/set/stop): `README.md`

**Checkpoint**: US1 independently testable (AP connect → AUTH → set forward → stop)

---

## Phase 4: User Story 2 - Reverse and Direction Control (Priority: P1)

**Goal**: Operator reverses direction with negative values and can stop reliably

**Independent Test**: After AUTH, send `{"action":"set","left":-150,"right":-150}` → verify backward motion; send `{"action":"stop"}` → verify stop

### Implementation for User Story 2

- [ ] T020 [P] Ensure negative inputs map correctly to DIR and duty magnitude: `src/motor_controller.cpp`
- [ ] T021 [P] Add saturation and guard against instant opposite flip (optional brief stop or ramp): `src/motor_controller.cpp`
- [ ] T022 Extend README with reverse examples and notes: `README.md`
- [ ] T023 Add serial logs for reverse cases (input→duty/dir mapping): `src/main.ino`

**Checkpoint**: US2 independently testable (reverse works safely)

---

## Phase 5: User Story 4 - Emergency Stop (Priority: P1)

**Goal**: Operator triggers emergency stop that immediately ceases all motion regardless of state

**Independent Test**: During motion, send `{"action":"estop"}` → immediate off; subsequent `set` rejected until cleared (if clear command supported) or after reset

### Implementation for User Story 4

- [ ] T024 Implement `estop` action; force outputs off immediately via wrapper: `src/command_parser.cpp`
- [ ] T025 Latch unsafe state in runtime (e.g., estop flag) and gate future set commands: `src/command_parser.cpp`
- [ ] T026 Add serial logs for estop/latch behavior: `src/main.ino`
- [ ] T027 Update README emergency stop usage: `README.md`

**Checkpoint**: US4 independently testable (estop dominates, immediate stop)

---

## Phase 6: User Story 5 - Invalid Command Handling (Priority: P1)

**Goal**: Malformed/unknown/out-of-range inputs produce clear error responses; no actuation occurs

**Independent Test**: Send invalid JSON, unknown action, missing fields, or out-of-range values; verify `status:error` and no motion

### Implementation for User Story 5

- [ ] T028 Implement strict schema validation (types/ranges) per contracts: `src/command_parser.cpp`
- [ ] T029 Add unauthenticated command rejection with clear code: `src/command_parser.cpp`
- [ ] T030 Add rate limiting for failed AUTH attempts (simple counter/backoff): `src/command_parser.cpp`
- [ ] T031 Ensure all errors return `{"status":"error","code":"…","message":"…"}`: `src/command_parser.cpp`
- [ ] T032 Extend README error examples and troubleshooting: `README.md`
- [ ] T033 Add serial error logging (parse failures, rejects), redacting tokens: `src/main.ino`

**Checkpoint**: US5 independently testable (robust error handling and feedback)

---

## Phase 7: User Story 3 - Independent Motor Control (Priority: P2)

**Goal**: Operator can independently set left and right motors (e.g., differential steering)

**Independent Test**: Send `{"action":"set","left":200,"right":50}` → verify turning motion; `stop` → verify stop

### Implementation for User Story 3

- [ ] T034 [P] Verify per-side mapping and apply both sides within a single command atomically: `src/command_parser.cpp`
- [ ] T035 [P] Add clamp/saturation per side; ensure 0 stops that side only: `src/motor_controller.cpp`
- [ ] T036 Document differential control examples in README: `README.md`
- [ ] T037 Add serial summaries for asymmetric set (L/R): `src/main.ino`

**Checkpoint**: US3 independently testable (independent L/R control)

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Docs, tests, and overall hardening

- [ ] T038 [P] Fill out `docs/motor_api.md` with full API reference (functions, params, returns, init order)
- [ ] T039 [P] Fill out `docs/gpio_wiring.md` with wiring diagrams, pin notes, sensor channel mapping
- [ ] T040 [P] Add basic serial-driven smoke tests in `tests/unit/` (e.g., mapping boundaries, estop dominance)
- [ ] T041 Security hardening: redact tokens in logs, confirm length policy, confirm single-client enforcement: `src/command_parser.cpp` + `src/main.ino`
- [ ] T042 Run quickstart validation: follow `specs/1-tcp-motor-control/quickstart.md` and adjust docs: `README.md`

---

## Phase 9: Sensors & Safety

**Purpose**: Implement ADS1115-based sensor sampling, safety_manager with retreat pattern table, telemetry/status, and calibration command

- [ ] T044 [P] Create ADS1115 sensor module (init on Wire1 GP26/GP27, address config): `src/sensors_ads1115.cpp` + `src/sensors_ads1115.h`
- [ ] T045 [P] Implement non-blocking read of 4 channels with debounce (5–20 ms) and threshold classification; treat I2C errors as unsafe: `src/sensors_ads1115.cpp`
- [ ] T046 [P] Implement safety_manager with retreat patterns and 3+ sensors → EMERGENCY_STOP rule; block unsafe set commands: `src/safety_manager.cpp` + `src/safety_manager.h`
- [ ] T047 [P] Implement telemetry/status payload per contract (auth, motors, sensors raw/flags, safety estop/latched): `src/telemetry.cpp` + `src/telemetry.h`
- [ ] T048 Implement `calibrate` command (auto mode) to compute/store thresholds in RAM and apply to sensor classification: `src/command_parser.cpp` + `src/sensors_ads1115.cpp`
- [ ] T049 Update docs with sensor channel mapping, retreat behaviors, and calibration steps: `README.md`

---

## Phase 10: PWM Edge-Case Tests (P7 Gate)

**Purpose**: Satisfy Constitution P7 by exercising boundary and out-of-range PWM inputs

- [ ] T050 [P] Implement boundary tests for mapping {-255, -1, 0, 1, 255}; assert duty and DIR mapping via serial logs or AUnit: `tests/unit/test_pwm_edges.cpp` (or serial test mode in `src/main.ino`)
- [ ] T051 [P] Implement out-of-range saturation tests for {-999, 999}; assert saturation and no unsafe motion: `tests/unit/test_pwm_edges.cpp` (or serial test mode)
- [ ] T052 Document P7 test procedure and expected outcomes; link to constitution: `docs/tests_pwm.md`

---

## Dependencies & Execution Order

### Phase Dependencies

- Setup (Phase 1): No dependencies - can start immediately
- Foundational (Phase 2): Depends on Setup completion - BLOCKS all user stories
- User Stories (Phases 3–7): Depend on Foundational completion
  - P1 stories (US1, US2, US4, US5) should be prioritized, with US1 as MVP
  - P2 story (US3) can start after Foundational, ideally after US1 for reuse
- Polish (Final Phase): Depends on selected user stories completed
 - Sensors & Safety (Phase 9): Depends on Foundational; best started after US1 to leverage established networking and motor wrapper
 - PWM Edge-Case Tests (Phase 10): Depends on Foundational and motor wrapper; can run after US1

### User Story Dependencies

- US1 (P1): No dependency on other stories; requires Foundational
- US2 (P1): No dependency on other stories; requires Foundational
- US4 (P1): No dependency on other stories; requires Foundational
- US5 (P1): No dependency on other stories; requires Foundational
- US3 (P2): No dependency on other stories; requires Foundational (best after US1)

### Within Each User Story

- Implement command handling in parser → invoke motor wrapper → observe via serial
- Keep loop non-blocking; avoid delay(); use millis-based scheduling
- Ensure JSONL responses meet contracts for ok/error

### Parallel Opportunities

- [P] tasks across different files can proceed in parallel:
  - Phase 2: T007–T012
  - Phase 2: T043 can be implemented alongside other networking tasks
  - US2: T020–T021
  - US3: T034–T035
  - Sensors & Safety: T044–T047 in parallel
  - Edge-Case Tests: T050–T051 in parallel
  - Polish: T038–T040

---

## Parallel Example: User Story 1

```text
# In parallel (different files):
- Implement WiFi AP/TCP accept/readline in src/main.ino (T013)
- Implement AUTH and set/stop in src/command_parser.cpp (T014–T015)
- Implement duty mapping and Motor init in src/motor_controller.cpp (T016–T017)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. STOP and VALIDATE: Test US1 independently (AP connect → AUTH → set forward → stop)
5. Demo/iterate

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add US1 → Test independently → Demo (MVP)
3. Add US2 + US4 + US5 (P1) → Test independently → Demo
4. Add US3 (P2) → Test independently → Demo

### Parallel Team Strategy

- After Foundational:
  - Developer A: US1 (network + auth + set/stop)
  - Developer B: US2 (reverse), US4 (estop)
  - Developer C: US5 (error handling), later US3 (independent control)

---

## Notes

- All motor actuation MUST go through existing Motor library via the wrapper; no new PWM implementations
- Status/error schemas must follow `specs/1-tcp-motor-control/contracts/jsonl-commands.md`
- Keep logs concise; redact tokens; prefer non-blocking
- Sensors/ADS1115 integration and safety pattern logic are covered in Phase 9; PWM edge-case tests required by P7 are covered in Phase 10.
