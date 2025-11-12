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