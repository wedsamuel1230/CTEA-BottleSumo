# Requirements Quality Checklist: TCP Motor Control (BottleSumo)

Purpose: Validate the quality of the specification and plan (completeness, clarity, consistency, measurability, coverage, dependencies, safety). These are unit tests for requirements, not implementation tests.

Created: 2025-10-25
Feature: ../spec.md | Plan: ../plan.md
Depth: Standard | Audience: Reviewer (PR)

## Requirement Completeness

- [ ] CHK001 Are all command actions (auth, set, stop, estop, status, calibrate) fully specified with fields and constraints? [Completeness, Spec §FR-012; Contracts/jsonl-commands.md]
- [ ] CHK002 Are motor control ranges and semantics (sign=direction, 0=stop) defined for both motors independently? [Completeness, Spec §FR-002, §FR-003]
- [ ] CHK003 Is safety behavior on session loss/inactivity explicitly defined with timing bounds? [Completeness, Spec §FR-005]
- [ ] CHK004 Are four QRE1113 sensors and their integration described end-to-end (sampling, debounce, thresholds, safety linkage)? [Completeness, Spec §FR-014, §FR-015, §FR-016]
- [ ] CHK005 Is the ADS1115 interface requirement (channels, address range, error handling) captured? [Completeness, Spec §FR-018]
- [ ] CHK006 Are non-functional constraints for non-blocking loop and sampling windows present? [Completeness, Spec §NFR-003]
- [ ] CHK007 Is I2C performance/latency requirement explicitly stated? [Completeness, Spec §NFR-004]
- [ ] CHK008 Is the authentication requirement and token policy fully captured (length, gating, failures)? [Completeness, Spec §FR-013; §NFR-002]
- [ ] CHK009 Is telemetry/status content defined (sensor raw/flags, auth, motion)? [Completeness, Spec §FR-017]
- [ ] CHK010 Does the plan include a complete GPIO map and wiring for all signals (2 PWM, 2 DIR, I2C), with no dedicated safety inputs per design? [Completeness, Plan §Hardware Design/ GPIO Map]

## Requirement Clarity

- [ ] CHK011 Are command field types and ranges unambiguous (e.g., left/right integers in [-255, 255])? [Clarity, Spec §FR-002; Contracts]
- [ ] CHK012 Is JSONL framing and response schema (status ok/error, codes) clearly defined? [Clarity, Spec §FR-012; Contracts]
- [ ] CHK013 Is the edge detection decision criterion (debounce window, threshold source) clear? [Clarity, Spec §FR-014, §FR-016; Edge Cases]
- [ ] CHK014 Is “immediate” safety reaction quantified by latency targets? [Clarity, Spec §SC-007]
- [ ] CHK015 Is the mapping from input speed to duty cycle precisely specified (including saturation behavior)? [Clarity, Spec §FR-002; Plan §Motor integration]
- [ ] CHK016 Are E‑STOP vs sensor-gating semantics clearly differentiated (latching vs gating)? [Clarity, Spec §FR-015; Plan §Safety manager]

## Requirement Consistency

- [ ] CHK017 Do motor control constraints consistently require reuse of Motor.h/.cpp across spec and plan (no alternate PWM)? [Consistency, Spec §FR-009; Plan §Dependencies & Inclusion]
- [ ] CHK018 Are success criteria aligned with non-functional constraints (SC-007 vs NFR-003/004)? [Consistency, Spec §SC-007; §NFR-003..004]
- [ ] CHK019 Is the sensor/I2C pinning consistent between plan and other project docs (e.g., Wire1 on GP26/27 as confirmed)? [Consistency, Plan §GPIO Map; Attachment BottleSumo_QRE1113_with_OLED_onstart.ino]
- [ ] CHK020 Are input pins’ active polarity (pull-ups, active-low) consistently documented between spec and plan? [Consistency, Plan §Hardware Design; Spec §Edge Cases]

## Acceptance Criteria Quality (Measurability)

- [ ] CHK021 Are PWM boundary/out-of-range acceptance criteria measurable and enumerated? [Acceptance Criteria, Spec §Edge Cases; §SC-002, §SC-003]
- [ ] CHK022 Is the safety latency target measurable with a clear method (timestamps or logging)? [Measurability, Spec §SC-007; Plan §Test Plan]
- [ ] CHK023 Is unauthenticated/invalid token behavior measurable (clear codes, retry/disconnect rules)? [Measurability, Spec §FR-013; §Edge Cases]
- [ ] CHK024 Are disconnect/inactivity safe-off behaviors measurable with defined timeouts? [Measurability, Spec §FR-005]

## Scenario Coverage

- [ ] CHK025 Do the user stories cover forward, reverse, independent control, emergency stop, invalid command handling? [Coverage, Spec §User Scenarios]
- [ ] CHK026 Are recovery/exception scenarios for I2C NACK/timeout/missing device defined to fail safe and recover? [Coverage, Spec §Edge Cases; §FR-018]
- [ ] CHK027 Are calibration flows covered (auto/manual, storage expectations)? [Coverage, Spec §FR-016]
- [ ] CHK028 Is status/telemetry scenario included for operator verification? [Coverage, Spec §FR-017]

## Edge Case Coverage

- [ ] CHK029 Are JSON parse errors and unknown actions explicitly handled with error responses and no actuation? [Edge Case, Spec §FR-012; §Edge Cases]
- [ ] CHK030 Are rapid direction flips constrained by safety rules (stop or ramp) to prevent damage? [Edge Case, Spec §Edge Cases]
- [ ] CHK031 Are sensor noise/ambient flicker handling requirements captured (debounce/filter window)? [Edge Case, Spec §Edge Cases; §NFR-003]
- [ ] CHK032 Is single-sensor failure (stuck high/low) addressed with safe default and health reporting? [Edge Case, Spec §Edge Cases]

## Non-Functional Requirements

- [ ] CHK033 Is the non-blocking architecture mandated and reflected in plan modules/scheduling? [NFR, Spec §NFR-003; Plan §Software Architecture]
- [ ] CHK034 Are Wi‑Fi reliability and safe defaults on disconnect documented? [NFR, Spec §FR-005; Plan §Risk Management]
- [ ] CHK035 Are security requirements for token length, logging hygiene, and rate limiting captured? [NFR/Security, Spec §NFR-002; §FR-013]
- [ ] CHK036 Is I2C data rate and bus config required to meet latency target without starving networking? [NFR, Spec §NFR-004]

## Dependencies & Assumptions

- [ ] CHK037 Are all required libraries and their roles listed (Arduino-Pico core, ArduinoJson, ADS1115 lib, Motor library)? [Dependencies, Plan §Dependencies & Inclusion]
- [ ] CHK038 Is the AP-mode networking (WiFi.h/WiFiServer) identified and aligned with constraints? [Dependencies, Spec §NFR-001; Plan §Software Architecture]
- [ ] CHK039 Are hardware protections/assumptions stated (series resistors, pull-ups, driver fault wiring if present)? [Assumptions, Plan §Hardware Design; §Risk Management]

## Ambiguities & Conflicts

- [ ] CHK040 Is there any conflict between example sketch I2C pinning (Wire1 GP26/27) and the plan’s GPIO map, and is the chosen mapping documented (Wire1 GP26/27)? [Conflict, Plan §GPIO Map; Attachment]
- [ ] CHK041 Is the latching/clear behavior for E‑STOP vs edge-latch explicitly defined (manual vs auto clear after dwell)? [Ambiguity, Spec §FR-015]
- [ ] CHK042 Is the single-client policy (or multi-client) explicitly documented to prevent command contention? [Ambiguity, Spec §FR-001; Plan §Wi‑Fi server]
- [ ] CHK043 Is the token provisioning/storage method defined (compile-time, EEPROM, or runtime)? [Ambiguity, Spec §FR-013; [Gap] if not specified]

## Traceability & IDs

- [ ] CHK044 Do requirements and acceptance criteria have identifiable references (FR/NFR/SC) allowing ≥80% coverage in this checklist? [Traceability, Spec all]
- [ ] CHK045 Is there a clear mapping from plan modules to spec requirements (e.g., sensors_ads1115 ↔ FR-014/FR-018)? [Traceability, Plan §Software Architecture]

---
Notes:
 - ADS1115 is confirmed on Wire1 (GP26 SDA, GP27 SCL); spec/plan/quickstart are aligned.
 - If multiple teams/tools coordinate calibration and testing, consider orchestration (MCP/agent) checklists in a separate run focusing on automation workflows.
