<!--
Sync Impact Report
- Version change: N/A → 1.0.0
- Modified principles: Replaced template placeholders with nine concrete principles (P1–P9)
- Added sections: Defined concrete "Additional Constraints & Safety Standards" and "Development Workflow & Quality Gates"
- Removed sections: None
- Templates requiring updates:
	- .specify/templates/plan-template.md → ✅ updated (Constitution Check gates materialized)
	- .specify/templates/spec-template.md → ✅ aligned (no changes required)
	- .specify/templates/tasks-template.md → ✅ aligned (no changes required)
	- .specify/templates/commands/*.md → ⚠ N/A (directory not present in repo)
- Follow-up TODOs:
	- Add LICENSE file declaring open-source license (e.g., MIT) consistent with Principle P8.
	- Add CONTRIBUTING.md and basic CODE_OF_CONDUCT.md to support collaboration.
-->

# CTEA-BottleSumo Robot Car Constitution
<!-- Project governance and development principles for the BottleSumo robot car -->

## Core Principles

### P1. Hardware Platform & GPIO Discipline (Pico W)
The robot MUST use the Raspberry Pi Pico W as the controller and adhere to a strict GPIO
allocation for dual brushless motor control: exactly 6 GPIO lines total — 2 PWM (one per
motor), 2 direction (one per motor), and 2 signal inputs (e.g., encoder or limit inputs).
Pin mapping MUST be documented in source and README diagrams; any change to mapping is a
governance amendment requiring explicit review.

Rationale: Fixed GPIO budgets prevent feature creep and wiring churn, enabling reliable,
repeatable builds across robots and competitions.

### P2. Network Role: AP-Mode TCP Server
The car operates as the access point (AP) and TCP server; external clients are controllers.
The firmware MUST expose a stable TCP command port with a clearly documented protocol.
Connection handling MUST include sane timeouts and a safe default state when no client is
connected or on disconnect.

Rationale: In competition, the car is the controlled device and must be directly reachable
without relying on external infrastructure.

### P3. Communication Security (Mandatory)
Only authorized clients may issue commands. The TCP endpoint MUST authenticate clients and
reject unauthorized or malformed requests. Commands MUST be validated against an allowlist,
inputs sanitized, and abuse mitigated (e.g., rate limiting or backoff). Security-relevant
events MUST be logged (at least in debug builds) without leaking secrets.

Rationale: Prevent hostile control, undefined behavior, or denial-of-service during matches.

### P4. Circuit Protection & Electrical Safety
All power and signal circuits MUST include basic protection suitable for the platform:
fuse or polyfuse on the motor rail, appropriate wire gauge, reverse-polarity protection,
series resistors or level shifting as required, and clearly labeled connectors.
An emergency stop (E‑STOP) and safe boot defaults (motors off) are REQUIRED.

Rationale: Protect hardware and bystanders; reduce damage from shorts, miswiring, or brownouts.

### P5. Maintainability & Documentation
Code MUST be modular, readable, and documented. Public APIs MUST be commented with purpose,
parameters, ranges, and error modes. Inline comments explain non-obvious logic. A minimal
README MUST document build, flash, wiring, and network usage once available. Formatting and
basic static checks SHOULD be run in CI or pre-commit when feasible.

Rationale: Maintainable code accelerates fixes and collaboration under time pressure.

### P6. Command Safety & Protective Limits
All control commands MUST be safety-checked to prevent hardware damage. Enforce bounds,
apply saturation, and use ramp/acceleration limiting as appropriate. On undervoltage,
overcurrent, sensor faults, or comms loss, the system MUST enter a safe state (motors off)
and recover only under controlled conditions.

Rationale: Software must never permit actions that risk hardware failure.

### P7. PWM Input Contract and Edge-Case Testing
All motion commands that map to PWM MUST accept the integer range [-255, 255]. 0 means stop;
sign encodes direction. Inputs outside the range MUST be saturated, not wrapped. A test
matrix MUST exercise at minimum: -255, -1, 0, 1, 255, and out-of-range values (e.g., -999,
999) to verify saturation and safety behavior.

Rationale: Edge-case coverage ensures predictable actuation and prevents overflow bugs.

### P8. Open Source & Collaboration
The project MUST be open source with a permissive license (e.g., MIT) added to the repo.
Contributions occur via pull requests with code review. Use semantic commits, issue
templates, and a CONTRIBUTING guide. Ownership and reviews MUST enforce this constitution.

Rationale: Enable community learning, reuse, and repeatable competition builds.

### P9. Motor Control Library Reuse (NON‑NEGOTIABLE)
All PWM operations MUST reuse the existing low-level motor control library in `Motor.h` and
`Motor.cpp`. PWM logic MUST NOT be reimplemented elsewhere. Higher-level motion code MAY wrap
or orchestrate the library but MUST call its exposed functions for PWM generation. Any change
to the low-level library requires review and broad regression tests.

Rationale: Centralizing PWM avoids drift, duplicated bugs, and timing regressions.

## Additional Constraints & Safety Standards

- Control loop timing and any blocking I/O MUST be documented; long operations use
	non-blocking patterns when feasible.
- On boot and on comms loss, motors MUST default to off within a bounded time (target <100 ms
	after fault detection).
- All wiring diagrams MUST clearly mark polarity and connector orientation to reduce
	miswiring risk.

## Development Workflow & Quality Gates

Every PR MUST pass a Constitution Check covering at least:
- P1: GPIO budget and pin mapping remain compliant and documented.
- P2: AP-mode TCP server behavior preserved; safe defaults on disconnect verified.
- P3: Authentication/authorization present; unauthorized commands are rejected.
- P4: Changes do not weaken circuit protection; E‑STOP path remains effective.
- P5: New/changed APIs documented; formatting/static checks pass if configured.
- P6: Command validation in place; fault handling leads to safe state.
- P7: Tests include PWM edge cases [-255..255] with saturation of out-of-range input.
- P8: OSS compliance maintained (license headers or repo license present).
- P9: All PWM routed through `Motor.h`/`Motor.cpp`; no duplicate PWM logic.

Merging without meeting these gates requires a documented exception with explicit approval
and a follow-up task to restore compliance.

## Governance

This constitution governs all development for the BottleSumo robot car. Amendments require
PRs that: (1) update this document, (2) include a migration/impact note, (3) specify a
version bump per semantic governance versioning: MAJOR for breaking/principle removals or
redefinitions; MINOR for added/expanded principles; PATCH for clarifications and non-semantic
wording fixes. Compliance reviews MUST verify Constitution Check gates in each PR.

**Version**: 1.0.0 | **Ratified**: 2025-10-25 | **Last Amended**: 2025-10-25
# [PROJECT_NAME] Constitution
<!-- Example: Spec Constitution, TaskFlow Constitution, etc. -->

## Core Principles

### [PRINCIPLE_1_NAME]
<!-- Example: I. Library-First -->
[PRINCIPLE_1_DESCRIPTION]
<!-- Example: Every feature starts as a standalone library; Libraries must be self-contained, independently testable, documented; Clear purpose required - no organizational-only libraries -->

### [PRINCIPLE_2_NAME]
<!-- Example: II. CLI Interface -->
[PRINCIPLE_2_DESCRIPTION]
<!-- Example: Every library exposes functionality via CLI; Text in/out protocol: stdin/args → stdout, errors → stderr; Support JSON + human-readable formats -->

### [PRINCIPLE_3_NAME]
<!-- Example: III. Test-First (NON-NEGOTIABLE) -->
[PRINCIPLE_3_DESCRIPTION]
<!-- Example: TDD mandatory: Tests written → User approved → Tests fail → Then implement; Red-Green-Refactor cycle strictly enforced -->

### [PRINCIPLE_4_NAME]
<!-- Example: IV. Integration Testing -->
[PRINCIPLE_4_DESCRIPTION]
<!-- Example: Focus areas requiring integration tests: New library contract tests, Contract changes, Inter-service communication, Shared schemas -->

### [PRINCIPLE_5_NAME]
<!-- Example: V. Observability, VI. Versioning & Breaking Changes, VII. Simplicity -->
[PRINCIPLE_5_DESCRIPTION]
<!-- Example: Text I/O ensures debuggability; Structured logging required; Or: MAJOR.MINOR.BUILD format; Or: Start simple, YAGNI principles -->

## [SECTION_2_NAME]
<!-- Example: Additional Constraints, Security Requirements, Performance Standards, etc. -->

[SECTION_2_CONTENT]
<!-- Example: Technology stack requirements, compliance standards, deployment policies, etc. -->

## [SECTION_3_NAME]
<!-- Example: Development Workflow, Review Process, Quality Gates, etc. -->

[SECTION_3_CONTENT]
<!-- Example: Code review requirements, testing gates, deployment approval process, etc. -->

## Governance
<!-- Example: Constitution supersedes all other practices; Amendments require documentation, approval, migration plan -->

[GOVERNANCE_RULES]
<!-- Example: All PRs/reviews must verify compliance; Complexity must be justified; Use [GUIDANCE_FILE] for runtime development guidance -->

**Version**: [CONSTITUTION_VERSION] | **Ratified**: [RATIFICATION_DATE] | **Last Amended**: [LAST_AMENDED_DATE]
<!-- Example: Version: 2.1.1 | Ratified: 2025-06-13 | Last Amended: 2025-07-16 -->
