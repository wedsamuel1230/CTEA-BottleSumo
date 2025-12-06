## [2025-11-18] Search Cadence + Sensor Validation Refresh

### Changed
- Slowed the blind-search rotation behavior by lowering `ROTATION_SPEED` and widening `BIAS_DEADZONE` so the car sweeps more gently when it loses the bottle.
- Tightened the default detection envelope (`DETECT_MAX_MM` now 800) to keep the robot focused on closer targets until the adaptive search logic expands the range.
- Relaxed side-cluster validation: each flank now tracks its own consistency counter and will accept any actionable sensor after two consecutive frames (`SIDE_CONFIRM_FRAMES = 2`), matching the “two readers” requirement without needing both sensors at once.
- Per-side confirmation frames reset automatically when echoes disappear, preventing stale confirmations from carrying over after a loss.

### Tuning Notes
- **Rotation feel:** Increase or decrease `ROTATION_SPEED` (default 40) to make the idle search spin faster or slower. Pair with `BIAS_DEADZONE` (default 0.25) to control how sensitive the robot is to small left/right differences before it commits to a turn.
- **Sensor strictness:** Adjust `SIDE_CONFIRM_FRAMES` (default 2) if you want more than two consecutive readings before acting, or drop it to 1 for instant reactions. `SIDE_CLOSE_PROX_MM` and `CLUSTER_DISTANCE_DELTA_MM` govern how tightly the side sensors must agree.
- **Search envelope:** `DETECT_MAX_MM`, `SEARCH_DISTANCE_STEP_MM`, and `SEARCH_EXPANSION_DELAY` define how quickly the adaptive range widens after you lose the opponent. Shorter delays or larger steps make the car look farther sooner.

## [2025-11-10] Major Feature - WiFi Telemetry Streaming

### Added
- WiFi Access Point mode (SSID: PicoW-CarTracker, 192.168.4.1)
- TCP telemetry server on port 8080 (JSON streaming @ 10Hz)
- TelemetryData struct with tracking metrics and sensor readings
- Telemetry mutex for thread-safe Core0→Core1 communication
- JSON serialization for real-time telemetry broadcast
- Multi-client support (up to 4 simultaneous TCP connections)
- Client connection management (automatic accept/disconnect handling)

### Changed
- Core1 now handles Motors + WiFi + TCP telemetry (tri-tasking)
- processTracking() now sends telemetry data to Core1
- setup1() extended with WiFi AP initialization and TCP server start
- loop1() extended with 10Hz telemetry broadcasting

### Performance
- Motor control still 50Hz (unchanged, no performance impact)
- Telemetry streaming at 10Hz (matches sensor reading rate)
- WiFi operations non-blocking (no motor control interference)
- JSON serialization ~2-5ms (acceptable within 100ms telemetry budget)

### Requirements
- **Hardware:** Raspberry Pi Pico W (WiFi-enabled) required
- **Library:** Arduino-Pico core with WiFi support

### Risk Level
- **Medium** - WiFi adds new dependency and complexity
- Thoroughly tested mutex implementation for telemetry
- Non-blocking WiFi operations prevent motor control interference
- Watchdog safety mechanisms still active
- Can disable WiFi by commenting out `setup1()` WiFi sections