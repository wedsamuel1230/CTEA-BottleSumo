# Data Model: TCP Motor Control

## Entities

### Session
- Fields:
  - id: string (implicit TCP connection)
  - authenticated: bool
  - lastActivityMs: uint32
- Rules:
  - inactivityTimeoutMs → safe-off when exceeded
  - only one active session allowed

### AuthState
- Fields:
  - tokenConfigured: bool
  - authAttempts: uint8 (rate limit)
- Rules:
  - token length ≥ 12
  - redact in logs

### MotorState
- Fields:
  - leftPwm: int [-255..255]
  - rightPwm: int [-255..255]
  - appliedDutyLeft: float [-100..100]
  - appliedDutyRight: float [-100..100]
- Rules:
  - mapIntToDuty(v): duty = clamp(v/255*100, -100..100)
  - zero → stop()

### SafetyState
- Fields:
  - estop: bool
  - edgeFrontLeft: bool
  - edgeFrontRight: bool
  - edgeRearLeft: bool
  - edgeRearRight: bool
  - unsafeLatched: bool
- Rules:
  - estop ⇒ unsafeLatched = true (clear only by power cycle or explicit clear if allowed)
  - if edge in intended direction ⇒ unsafeLatched = true until cleared (dwell ok)

### SensorReading
- Fields:
  - raw[4]: uint16 (ADS1115)
  - thresh[4]: uint16
  - flags[4]: bool (edge)
- Rules:
  - debounceWindowMs: 5–20
  - classify(raw[i], thresh[i])

### Command (JSONL)
- Fields:
  - action: enum {auth,set,stop,estop,status,calibrate}
  - params: object per action
- Rules:
  - validate schema per contracts
  - reject unknown/invalid; never actuate on error

## State Transitions (Safety)

- any → estop: motors off immediately; unsafeLatched = true
- moving → edge(direction): motors off; unsafeLatched = true; block set toward edge
- unsafeLatched → clear: when all sensors safe and dwell elapsed; estop requires manual clear
