# JSON Lines Command Contract

Each request and response is a single JSON object terminated by a newline. Unknown actions or invalid schema MUST return an error and MUST NOT actuate motors.

## Common Response

```json
{"status":"ok"}
```

```json
{"status":"error","code":"ERR_CODE","message":"details"}
```

## Actions

### auth
Request:
```json
{"action":"auth","token":"0123456789AB"}
```
Responses:
- ok on success
- error codes: AUTH_REQUIRED, AUTH_INVALID, AUTH_RATE_LIMIT

### set
Request:
```json
{"action":"set","left":-255,"right":200}
```
Rules:
- left/right integers in [-255,255]
- Requires authenticated session
- Blocked if unsafe would result (e.g., front edge + forward command)

### stop
Request:
```json
{"action":"stop"}
```
Effect: both motors off

### estop
Request:
```json
{"action":"estop"}
```
Effect: immediate off, latch unsafe

### status
Request:
```json
{"action":"status"}
```
Response ok + payload fields:
```json
{
  "status":"ok",
  "auth":true,
  "motors":{"left":0,"right":0},
  "sensors":{"raw":[123,456,789,100],"flags":[false,false,false,false]},
  "safety":{"estop":false,"latched":false}
}
```

### calibrate
Request:
```json
{"action":"calibrate","mode":"auto","samples":64}
```
Effect: capture baselines and compute thresholds; store in RAM (and optionally EEPROM if available)

## Error Codes
- AUTH_REQUIRED
- AUTH_INVALID
- AUTH_RATE_LIMIT
- BAD_REQUEST
- OUT_OF_RANGE
- UNSAFE_STATE
- INTERNAL_ERROR

## Safety behavior mapping (reference)

The device enforces safety automatically based on sensor flags. For client clarity, the default retreat vectors are:

- Channel mapping: A0 = front-left, A1 = front-right, A2 = rear-left, A3 = rear-right. Flags form a 4-bit pattern [A3 A2 A1 A0].
- Motor speed values are duty percentages [-100..100]; the firmware maps these to the Motor library outputs.

| Pattern (Binary) | Hex | Sensors Detected         | Action                    | Motor Speeds (L, R) |
|------------------|-----|--------------------------|---------------------------|---------------------|
| `0b0001`         | 0x1 | A0 only                  | RETREAT_BACKWARD_RIGHT    | (-80, -40)          |
| `0b0010`         | 0x2 | A1 only                  | RETREAT_BACKWARD_LEFT     | (-40, -80)          |
| `0b0100`         | 0x4 | A2 only                  | RETREAT_FORWARD_RIGHT     | (80, 40)            |
| `0b1000`         | 0x8 | A3 only                  | RETREAT_FORWARD_LEFT      | (40, 80)            |
| `0b0011`         | 0x3 | A0+A1 (front/top)        | RETREAT_BACKWARD          | (-80, -80)          |
| `0b1100`         | 0xC | A2+A3 (rear/bottom)      | RETREAT_FORWARD           | (80, 80)            |
| `0b0101`         | 0x5 | A0+A2 (left side)        | RETREAT_RIGHT             | (60, -60)           |
| `0b1010`         | 0xA | A1+A3 (right side)       | RETREAT_LEFT              | (-60, 60)           |
| `0b1001`         | 0x9 | A0+A3 (diagonal)         | RETREAT_FORWARD_RIGHT     | (80, 40)            |
| `0b0110`         | 0x6 | A1+A2 (diagonal)         | RETREAT_FORWARD_LEFT      | (40, 80)            |

Emergency Stop: Any pattern with three or more sensors active (≥3 bits set) triggers EMERGENCY_STOP (motors off immediately).
