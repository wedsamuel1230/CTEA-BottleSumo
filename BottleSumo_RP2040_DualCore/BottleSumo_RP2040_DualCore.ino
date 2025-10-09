/*
 * ======================================================================
 * RP2040 Dual-Core Bottle Sumo Robot Firmware
 * ======================================================================
 * 
 * ARCHITECTURE OVERVIEW:
 * ----------------------
 * Core 1 (Acquisition): High-frequency sensor polling, data processing,
 *                       lockless snapshot publishing, telemetry generation
 * Core 0 (State Machine): Snapshot consumption, tactical decision-making,
 *                         motor command generation, WiFi/TCP management
 * 
 * LOCKLESS COMMUNICATION:
 * -----------------------
 * - Double-buffered SensorSnapshot (Core1 write, Core0 read)
 * - Memory barriers ensure visibility across cores
 * - CommandBlock (Core0 write, Core1 read) for motor + threshold updates
 * 
 * TIMING SPECIFICATIONS:
 * ----------------------
 * Core 1 Tasks (millis-based scheduler):
 *   - IR Acquisition:      10 ms period (100 Hz)
 *   - Button Sampling:      5 ms period (200 Hz)
 *   - Button Debouncing:   20 ms period (50 Hz)
 *   - ToF Polling:         20 ms period per sensor (50 Hz each, staggered)
 *   - Snapshot Publish:    10 ms period (100 Hz)
 *   - Telemetry JSON:      50 ms period (20 Hz)
 *   - Command Poll:        Every loop (~500 Hz)
 *   - Statistics:        1000 ms period (1 Hz)
 * 
 * Core 0 Tasks (main loop):
 *   - Snapshot Fetch:      Every loop (~100 Hz)
 *   - State Machine:       Every loop (~100 Hz)
 *   - Command Update:      As needed (event-driven)
 *   - WiFi/TCP:            25-50 ms throttled
 * 
 * MEMORY CONSTRAINTS:
 * -------------------
 * - RP2040: 264 KB RAM total
 * - Core0 Stack: ~128 KB
 * - Core1 Stack: ~128 KB
 * - Static Allocation ONLY (no malloc/free in hot paths)
 * 
 * HARDWARE CONFIGURATION:
 * -----------------------
 * I2C Buses:
 *   - Wire (GP4/GP5):   ADS1115 ADC @ 0x48 (IR sensors)
 *   - Wire1 (GP26/GP27): VL53L0X ToF @ 0x30/0x31/0x32, OLED @ 0x3C
 * 
 * ToF Sensors:
 *   - XSHUT Pins: GP11 (RIGHT), GP12 (FRONT), GP13 (LEFT)
 *   - Continuous mode: 20-25 ms timing budget per sensor
 *   - Staggered startup: 0 ms / 7 ms / 14 ms offsets
 * 
 * IR Sensors (ADS1115):
 *   - Channel 0: Front-Left QRE1113
 *   - Channel 1: Front-Right QRE1113
 *   - Channel 2: Back-Left QRE1113
 *   - Channel 3: Back-Right QRE1113
 *   - Gain: ONE (±4.096V), SPS: 860
 * 
 * Buttons (Future Expansion):
 *   - Hardware TBD - debouncing logic stubbed
 * 
 * COMPILE TARGET:
 * ---------------
 * - Board: Raspberry Pi Pico W
 * - Framework: arduino-pico (Earle F. Philhower III)
 * - CPU Frequency: 125 MHz (default)
 * - Flash: 2 MB (w/ generic SPI)
 * 
 * AUTHOR: CTEA-BottleSumo Project (Autonomous Copilot Agent)
 * DATE: 2025-10-06
 * VERSION: 1.0.0 (Initial RP2040 Dual-Core Architecture)
 * 
 * ======================================================================
 */

// ======================================================================
// SECTION 1: LIBRARY INCLUDES
// ======================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>
#include <pico/mutex.h>
#include <hardware/sync.h>

// ======================================================================
// SECTION 2: CONFIGURATION CONSTANTS
// ======================================================================

namespace Config {
  // ----- Hardware Pin Assignments -----
  constexpr uint8_t TOF_XSHUT_RIGHT = 11;   // GP11
  constexpr uint8_t TOF_XSHUT_FRONT = 12;   // GP12
  constexpr uint8_t TOF_XSHUT_LEFT = 13;    // GP13
  
  constexpr uint8_t OLED_SDA_PIN = 26;      // GP26
  constexpr uint8_t OLED_SCL_PIN = 27;      // GP27
  
  // Button pins (TBD - not yet implemented in hardware)
  constexpr uint8_t BUTTON_PIN_START = 14;  // GP14 (example)
  constexpr uint8_t BUTTON_PIN_STOP = 15;   // GP15 (example)
  constexpr uint8_t BUTTON_COUNT = 2;       // Expandable to 8
  
  // ----- I2C Addresses -----
  constexpr uint8_t ADS1115_ADDRESS = 0x48;
  constexpr uint8_t TOF_ADDRESS_RIGHT = 0x30;
  constexpr uint8_t TOF_ADDRESS_FRONT = 0x31;
  constexpr uint8_t TOF_ADDRESS_LEFT = 0x32;
  constexpr uint8_t OLED_ADDRESS = 0x3C;
  
  // ----- Sensor Counts -----
  constexpr uint8_t IR_SENSOR_COUNT = 4;
  constexpr uint8_t TOF_SENSOR_COUNT = 3;
  
  // ----- Timing Periods (milliseconds) -----
  constexpr uint32_t IR_PERIOD_MS = 10;          // 100 Hz IR acquisition
  constexpr uint32_t BUTTON_SAMPLE_MS = 5;       // 200 Hz button sampling
  constexpr uint32_t BUTTON_DEBOUNCE_MS = 20;    // 50 Hz debounce processing
  constexpr uint32_t TOF_PERIOD_MS = 20;         // 50 Hz per ToF sensor
  constexpr uint32_t TOF_STAGGER_OFFSET_MS = 7;  // Stagger ToF sensors by 7ms
  constexpr uint32_t SNAPSHOT_PERIOD_MS = 10;    // 100 Hz snapshot publish
  constexpr uint32_t TELEMETRY_PERIOD_MS = 50;   // 20 Hz JSON telemetry
  constexpr uint32_t STATS_PERIOD_MS = 1000;     // 1 Hz statistics
  
  // ----- ToF Sensor Timing -----
  constexpr uint32_t TOF_TIMING_BUDGET_US = 25000;  // 25 ms per reading
  constexpr uint8_t TOF_VCSEL_PRE_RANGE = 14;
  constexpr uint8_t TOF_VCSEL_FINAL_RANGE = 10;
  constexpr uint32_t TOF_STALE_MS = 150;           // Data freshness tolerance
  
  // ----- ToF Detection Thresholds -----
  constexpr uint16_t DETECT_THRESH_MM = 1600;      // 160 cm opponent detection
  constexpr uint16_t TOF_MAX_VALID_RANGE_MM = 1500;
  constexpr uint16_t TOF_MIN_VALID_RANGE_MM = 30;
  constexpr uint8_t TOF_MAX_VALID_STATUS = 2;
  
  // ----- IR Sensor Thresholds -----
  constexpr float IR_EDGE_THRESHOLD_DEFAULT = 2.5f;  // Volts
  constexpr float IR_EDGE_THRESHOLD_MIN = 0.1f;
  constexpr float IR_EDGE_THRESHOLD_MAX = 4.0f;
  constexpr uint32_t IR_STALE_MS = 50;               // Data freshness tolerance
  
  // ----- ADS1115 Configuration -----
  constexpr uint16_t ADS1115_SPS = 860;              // Samples per second
  constexpr float ADS1115_GAIN_MULTIPLIER = 0.125f;  // mV per bit (GAIN_ONE)
  constexpr float MILLIVOLTS_TO_VOLTS = 0.001f;
  
  // ----- Button Debouncing -----
  constexpr uint8_t DEBOUNCE_SAMPLES_REQUIRED = 4;   // 4 samples @ 5ms = 20ms
  
  // ----- Staleness Thresholds (Core0 Safety) -----
  constexpr uint32_t SNAPSHOT_STALE_MS = 40;         // Safety fallback trigger
  
  // ----- Telemetry Buffer -----
  constexpr uint16_t TELEMETRY_BUFFER_SIZE = 512;    // Bytes for JSON string
  
  // ----- WiFi Configuration (Stub for Future) -----
  constexpr const char* WIFI_AP_SSID = "BottleSumo_Robot";
  constexpr const char* WIFI_AP_PASSWORD = "sumo2025";
  constexpr uint16_t TCP_SERVER_PORT = 4242;
  constexpr uint8_t MAX_TCP_CLIENTS = 6;
  
  // ----- Status Flags Bitmasks -----
  constexpr uint16_t STATUS_IR_STALE = (1 << 0);
  constexpr uint16_t STATUS_TOF_STALE = (1 << 1);
  constexpr uint16_t STATUS_TOF_OFFLINE = (1 << 2);
  constexpr uint16_t STATUS_COMMAND_UPDATED = (1 << 3);
}

// ======================================================================
// SECTION 3: DATA STRUCTURES & ENUMERATIONS
// ======================================================================

// ----- Edge Direction Enumeration -----
// Represents the direction of detected edge/white line
enum class EdgeDirection : uint8_t {
  SAFE = 0,        // No edge detected
  FRONT,           // Both front sensors detect edge
  BACK,            // Both back sensors detect edge
  LEFT,            // Left side sensors detect edge
  RIGHT,           // Right side sensors detect edge
  FRONT_LEFT,      // Front-left corner
  FRONT_RIGHT,     // Front-right corner
  BACK_LEFT,       // Back-left corner
  BACK_RIGHT       // Back-right corner
};

// ----- Sensor Snapshot Structure -----
// Contains all sensor data and derived metrics for one acquisition cycle
// Size: ~128 bytes (aligned for cache efficiency)
struct SensorSnapshot {
  // ----- Sequence & Timestamps -----
  uint32_t sequence;            // Monotonic sequence number (atomic increment)
  uint32_t captureMillis;       // millis() when snapshot was captured
  uint32_t irTimestamp;         // millis() of most recent IR read
  uint32_t tofTimestamp;        // millis() of most recent ToF read
  
  // ----- IR Sensor Data (4 channels) -----
  int16_t irRaw[Config::IR_SENSOR_COUNT];       // ADC raw values (0-32767)
  float irVolts[Config::IR_SENSOR_COUNT];       // Converted voltages (0-4.096V)
  float thresholds[Config::IR_SENSOR_COUNT];    // Threshold snapshot (per-sensor)
  
  // ----- Derived IR Metrics -----
  uint8_t dangerLevel;          // Count of sensors over threshold (0-4)
  bool edgeDetected;            // True if any sensor exceeds threshold
  EdgeDirection edgeDir;        // Direction of detected edge
  
  // ----- ToF Sensor Data (3 sensors: RIGHT, FRONT, LEFT) -----
  uint16_t tofDist[Config::TOF_SENSOR_COUNT];   // Distance in mm (0 if invalid)
  uint8_t tofValidMask;         // Bit mask: bit[i] = 1 if tofDist[i] valid
  uint8_t tofStatus[Config::TOF_SENSOR_COUNT];  // VL53L0X status codes
  
  // ----- Opponent Detection -----
  uint8_t opponentDirMask;      // Bit mask: bit0=FRONT, bit1=RIGHT, bit2=LEFT
  
  // ----- Button State (Future Expansion) -----
  uint8_t buttonsStableMask;    // Debounced stable button states (1=pressed)
  uint8_t buttonsEdgeMask;      // Rising edges since last snapshot
  
  // ----- Status Flags -----
  uint16_t statusFlags;         // System health flags (see Config::STATUS_*)
  
  // ----- Reserved for Alignment -----
  uint8_t reserved[6];          // Padding to 128-byte boundary
  
  // Constructor: Initialize to safe defaults
  SensorSnapshot() : 
    sequence(0), captureMillis(0), irTimestamp(0), tofTimestamp(0),
    dangerLevel(0), edgeDetected(false), edgeDir(EdgeDirection::SAFE),
    tofValidMask(0), opponentDirMask(0),
    buttonsStableMask(0), buttonsEdgeMask(0), statusFlags(0)
  {
    for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
      irRaw[i] = 0;
      irVolts[i] = 0.0f;
      thresholds[i] = Config::IR_EDGE_THRESHOLD_DEFAULT;
    }
    for (int i = 0; i < Config::TOF_SENSOR_COUNT; i++) {
      tofDist[i] = 0;
      tofStatus[i] = 0xFF;  // Invalid status
    }
    memset(reserved, 0, sizeof(reserved));
  }
};

// ----- Sensor Snapshot Exchange (Lockless Double-Buffer) -----
// Core1 publishes to inactive buffer, then flips publishIndex
// Core0 reads from active buffer indicated by publishIndex
struct SensorSnapshotExchange {
  SensorSnapshot buffers[2];           // Double buffer
  volatile uint8_t publishIndex;       // 0 or 1 (index of latest complete buffer)
  volatile uint32_t latestSequence;    // Sequence number of latest published snapshot
  
  // Constructor
  SensorSnapshotExchange() : publishIndex(0), latestSequence(0) {}
};

// ----- Command Block (Core0 → Core1 Communication) -----
// Core0 writes motor commands and threshold updates
// Core1 polls for sequence changes and applies commands
struct CommandBlock {
  volatile uint32_t seq;                 // Command sequence number (increment on write)
  
  // Motor Commands
  int16_t motorLeft;                     // Left motor PWM (-255 to +255)
  int16_t motorRight;                    // Right motor PWM (-255 to +255)
  
  // Threshold Updates
  float thresholds[Config::IR_SENSOR_COUNT];  // New threshold values
  uint8_t thresholdMask;                 // Bitmask: bit[i]=1 to update thresholds[i]
  
  // Command Flags
  uint16_t flags;                        // Future expansion (e.g., MODE_CHANGE)
  
  // Constructor
  CommandBlock() : seq(0), motorLeft(0), motorRight(0), thresholdMask(0), flags(0) {
    for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
      thresholds[i] = Config::IR_EDGE_THRESHOLD_DEFAULT;
    }
  }
};

// ======================================================================
// SECTION 4: GLOBAL INSTANCES
// ======================================================================

// ----- Sensor Snapshot Exchange (Core1 → Core0) -----
SensorSnapshotExchange g_sensorExchange;

// ----- Command Block (Core0 → Core1) -----
CommandBlock g_commandBlock;

// ----- Hardware Instances -----
Adafruit_ADS1115 g_ads;                          // ADS1115 ADC for IR sensors
Adafruit_VL53L0X g_loxRight, g_loxFront, g_loxLeft;  // VL53L0X ToF sensors

// ----- Mutexes (for Wire1 I2C bus shared by ToF + OLED) -----
mutex_t g_wire1Mutex;                            // Protects Wire1 bus access

// ----- Telemetry Buffer (Core1) -----
char g_telemetryBuffer[Config::TELEMETRY_BUFFER_SIZE];

// ----- Core1 State -----
volatile uint32_t g_core1LoopCount = 0;          // Performance counter
volatile bool g_core1Ready = false;              // Initialization complete flag

// ----- Core0 State -----
volatile uint32_t g_core0LoopCount = 0;          // Performance counter

// ----- ToF Sensor Initialization Status -----
bool g_tofInitialized[Config::TOF_SENSOR_COUNT] = {false, false, false};
uint32_t g_tofPerSensorTimestamp[Config::TOF_SENSOR_COUNT] = {0, 0, 0};

// ----- Button Debouncing State (Core1) -----
struct ButtonDebounceState {
  uint8_t rawHistory[Config::BUTTON_COUNT][Config::DEBOUNCE_SAMPLES_REQUIRED];
  uint8_t historyIndex;
  uint8_t stableMask;
  uint8_t edgeMask;
  
  ButtonDebounceState() : historyIndex(0), stableMask(0), edgeMask(0) {
    memset(rawHistory, 0, sizeof(rawHistory));
  }
};
ButtonDebounceState g_buttonState;

// ======================================================================
// SECTION 5: UTILITY MACROS & INLINE FUNCTIONS
// ======================================================================

// ----- Memory Barrier (ARM Cortex-M0+ Data Memory Barrier) -----
// Ensures all memory writes are visible to other cores before proceeding
#define DMB() __asm__ volatile("dmb" ::: "memory")

// ----- Bit Manipulation Macros -----
#define SET_BIT(value, bit)    ((value) |= (1U << (bit)))
#define CLEAR_BIT(value, bit)  ((value) &= ~(1U << (bit)))
#define TEST_BIT(value, bit)   (((value) >> (bit)) & 1U)

// ----- Inline Helpers -----

// Clamp value to range [min, max]
inline float clampf(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

// Check if timestamp is within freshness tolerance
inline bool isTimestampFresh(uint32_t timestamp, uint32_t tolerance_ms) {
  return (millis() - timestamp) < tolerance_ms;
}

// ======================================================================
// SECTION 6: LOCKLESS PUBLISH/SUBSCRIBE PRIMITIVES
// ======================================================================

// ----- publishSensorSnapshot (Core1 Only) -----
// Publishes a complete sensor snapshot to the exchange buffer
// Uses double-buffering to avoid blocking Core0 reads
// 
// WCET: ~10 µs (memory copy + atomic operations)
//
// Algorithm:
// 1. Copy draft snapshot to inactive buffer
// 2. Increment sequence number
// 3. Memory barrier (ensure writes visible to Core0)
// 4. Flip publishIndex to make buffer active
// 5. Update latestSequence
//
void publishSensorSnapshot(const SensorSnapshot& draft) {
  // Determine inactive buffer index
  uint8_t currentIndex = g_sensorExchange.publishIndex;
  uint8_t inactiveIndex = 1 - currentIndex;
  
  // Copy draft to inactive buffer
  memcpy(&g_sensorExchange.buffers[inactiveIndex], &draft, sizeof(SensorSnapshot));
  
  // Memory barrier: ensure all writes to inactive buffer complete
  DMB();
  
  // Atomically flip publishIndex to make inactive buffer active
  g_sensorExchange.publishIndex = inactiveIndex;
  
  // Update sequence (Core0 can detect new data by comparing sequence)
  g_sensorExchange.latestSequence = draft.sequence;
  
  // Final memory barrier
  DMB();
}

// ----- fetchLatestSnapshot (Core0 Only) -----
// Fetches the latest sensor snapshot from the exchange buffer
// Retry logic handles race condition if buffer flips mid-copy
//
// WCET: ~20 µs (memory copy with retry)
//
// Algorithm:
// 1. Read current publishIndex
// 2. Copy active buffer to output
// 3. Re-read publishIndex
// 4. If index changed, retry once (handles race condition)
//
// Returns: true if snapshot successfully fetched, false if retry failed
//
bool fetchLatestSnapshot(SensorSnapshot& out) {
  for (int attempt = 0; attempt < 2; attempt++) {
    // Read current active buffer index
    uint8_t startIndex = g_sensorExchange.publishIndex;
    DMB();  // Ensure index read is fresh
    
    // Copy active buffer
    memcpy(&out, &g_sensorExchange.buffers[startIndex], sizeof(SensorSnapshot));
    DMB();  // Ensure copy completes before re-reading index
    
    // Check if publishIndex changed during copy (race condition)
    uint8_t endIndex = g_sensorExchange.publishIndex;
    if (startIndex == endIndex) {
      return true;  // Successful read
    }
    
    // Index changed - retry once
  }
  
  // Failed after 2 attempts (extremely rare)
  return false;
}

// ======================================================================
// SECTION 7: MODULE A - ToF CONTINUOUS MODE
// ======================================================================

// ----- initToFSensors -----
// Initialize 3 VL53L0X sensors with unique I2C addresses
// Uses XSHUT pins to sequentially wake sensors and reassign addresses
//
// WCET: ~500 ms (blocking I2C transactions during startup)
//
// Returns: true if all sensors initialized successfully
//
bool initToFSensors() {
  Serial.println("=== ToF Sensor Initialization (Continuous Mode) ===");
  
  // Reset all sensors (XSHUT low)
  pinMode(Config::TOF_XSHUT_RIGHT, OUTPUT);
  pinMode(Config::TOF_XSHUT_FRONT, OUTPUT);
  pinMode(Config::TOF_XSHUT_LEFT, OUTPUT);
  digitalWrite(Config::TOF_XSHUT_RIGHT, LOW);
  digitalWrite(Config::TOF_XSHUT_FRONT, LOW);
  digitalWrite(Config::TOF_XSHUT_LEFT, LOW);
  delay(10);  // Ensure sensors enter reset
  
  // --- Initialize RIGHT sensor (0x30) ---
  digitalWrite(Config::TOF_XSHUT_RIGHT, HIGH);
  delay(10);
  if (!g_loxRight.begin(Config::TOF_ADDRESS_RIGHT, false, &Wire1)) {
    Serial.println("ERROR: RIGHT ToF init failed");
    g_tofInitialized[0] = false;
    return false;
  }
  g_loxRight.setMeasurementTimingBudgetMicroSeconds(Config::TOF_TIMING_BUDGET_US);
  g_loxRight.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, Config::TOF_VCSEL_PRE_RANGE);
  g_loxRight.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Config::TOF_VCSEL_FINAL_RANGE);
  g_loxRight.startRangeContinuous();  // Start continuous ranging mode
  g_tofInitialized[0] = true;
  Serial.println("RIGHT ToF: OK @ 0x30 (continuous mode)");
  
  // --- Initialize FRONT sensor (0x31) ---
  delay(Config::TOF_STAGGER_OFFSET_MS);  // Stagger startup by 7ms
  digitalWrite(Config::TOF_XSHUT_FRONT, HIGH);
  delay(10);
  if (!g_loxFront.begin(Config::TOF_ADDRESS_FRONT, false, &Wire1)) {
    Serial.println("ERROR: FRONT ToF init failed");
    g_tofInitialized[1] = false;
    return false;
  }
  g_loxFront.setMeasurementTimingBudgetMicroSeconds(Config::TOF_TIMING_BUDGET_US);
  g_loxFront.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, Config::TOF_VCSEL_PRE_RANGE);
  g_loxFront.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Config::TOF_VCSEL_FINAL_RANGE);
  g_loxFront.startRangeContinuous();
  g_tofInitialized[1] = true;
  Serial.println("FRONT ToF: OK @ 0x31 (continuous mode)");
  
  // --- Initialize LEFT sensor (0x32) ---
  delay(Config::TOF_STAGGER_OFFSET_MS);  // Stagger startup by another 7ms
  digitalWrite(Config::TOF_XSHUT_LEFT, HIGH);
  delay(10);
  if (!g_loxLeft.begin(Config::TOF_ADDRESS_LEFT, false, &Wire1)) {
    Serial.println("ERROR: LEFT ToF init failed");
    g_tofInitialized[2] = false;
    return false;
  }
  g_loxLeft.setMeasurementTimingBudgetMicroSeconds(Config::TOF_TIMING_BUDGET_US);
  g_loxLeft.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, Config::TOF_VCSEL_PRE_RANGE);
  g_loxLeft.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Config::TOF_VCSEL_FINAL_RANGE);
  g_loxLeft.startRangeContinuous();
  g_tofInitialized[2] = true;
  Serial.println("LEFT ToF: OK @ 0x32 (continuous mode)");
  
  Serial.println("All ToF sensors initialized successfully!");
  return true;
}

// ----- pollOneToFSensor -----
// Polls a single ToF sensor in continuous mode (non-blocking)
// Only reads data if sensor indicates data is ready
//
// WCET: ~1-2 ms if data ready, <100 µs if not ready
//
// Parameters:
//   - sensorIndex: 0=RIGHT, 1=FRONT, 2=LEFT
//   - snapshot: SensorSnapshot to update with new data
//
void pollOneToFSensor(uint8_t sensorIndex, SensorSnapshot& snapshot) {
  if (!g_tofInitialized[sensorIndex]) {
    // Sensor offline - mark as invalid
    snapshot.tofDist[sensorIndex] = 0;
    CLEAR_BIT(snapshot.tofValidMask, sensorIndex);
    snapshot.tofStatus[sensorIndex] = 0xFF;
    SET_BIT(snapshot.statusFlags, Config::STATUS_TOF_OFFLINE);
    return;
  }
  
  // Select sensor object
  Adafruit_VL53L0X* lox = nullptr;
  if (sensorIndex == 0) lox = &g_loxRight;
  else if (sensorIndex == 1) lox = &g_loxFront;
  else if (sensorIndex == 2) lox = &g_loxLeft;
  
  if (lox == nullptr) return;
  
  // Check if new data is ready (non-blocking)
  if (!lox->isRangeComplete()) {
    return;  // No new data yet
  }
  
  // Lock Wire1 mutex (ToF shares bus with OLED)
  mutex_enter_blocking(&g_wire1Mutex);
  
  // Read distance measurement
  VL53L0X_RangingMeasurementData_t measure;
  lox->rangingTest(&measure, false);
  
  mutex_exit(&g_wire1Mutex);
  
  // Update per-sensor timestamp
  g_tofPerSensorTimestamp[sensorIndex] = millis();
  
  // Validate measurement
  bool valid = (measure.RangeStatus <= Config::TOF_MAX_VALID_STATUS) &&
               (measure.RangeMilliMeter >= Config::TOF_MIN_VALID_RANGE_MM) &&
               (measure.RangeMilliMeter < Config::TOF_MAX_VALID_RANGE_MM);
  
  snapshot.tofDist[sensorIndex] = valid ? measure.RangeMilliMeter : 0;
  snapshot.tofStatus[sensorIndex] = measure.RangeStatus;
  
  if (valid) {
    SET_BIT(snapshot.tofValidMask, sensorIndex);
  } else {
    CLEAR_BIT(snapshot.tofValidMask, sensorIndex);
  }
  
  // Update composite ToF timestamp (max of all sensor timestamps)
  uint32_t maxTimestamp = 0;
  for (int i = 0; i < Config::TOF_SENSOR_COUNT; i++) {
    if (g_tofPerSensorTimestamp[i] > maxTimestamp) {
      maxTimestamp = g_tofPerSensorTimestamp[i];
    }
  }
  snapshot.tofTimestamp = maxTimestamp;
  
  // Opponent detection: if valid and close, set direction bit
  if (valid && measure.RangeMilliMeter < Config::DETECT_THRESH_MM) {
    SET_BIT(snapshot.opponentDirMask, sensorIndex);
  } else {
    CLEAR_BIT(snapshot.opponentDirMask, sensorIndex);
  }
}

// ======================================================================
// SECTION 8: MODULE B - BUTTON DEBOUNCING (STUB)
// ======================================================================

// ----- sampleButtonsRaw -----
// Sample raw button states and store in history buffer
// Call every 5 ms (BUTTON_SAMPLE_MS)
//
// WCET: <50 µs
//
// NOTE: This is a STUB implementation - actual pins TBD
//       Uncomment and configure pins when hardware is available
//
void sampleButtonsRaw() {
  // TODO: Implement when button hardware is added to robot
  // Example implementation:
  /*
  uint8_t rawMask = 0;
  if (digitalRead(Config::BUTTON_PIN_START) == LOW) {  // Active low
    SET_BIT(rawMask, 0);
  }
  if (digitalRead(Config::BUTTON_PIN_STOP) == LOW) {
    SET_BIT(rawMask, 1);
  }
  
  // Store in history buffer
  for (int i = 0; i < Config::BUTTON_COUNT; i++) {
    g_buttonState.rawHistory[i][g_buttonState.historyIndex] = TEST_BIT(rawMask, i);
  }
  */
  
  // STUB: No-op until hardware exists
}

// ----- debounceButtons -----
// Process debouncing logic for all buttons
// Call every 20 ms (BUTTON_DEBOUNCE_MS)
//
// WCET: <100 µs
//
// Algorithm:
//   - A button is "stable" if it has the same raw value for 4 consecutive samples (20ms)
//   - Edge mask captures transitions since last call
//
void debounceButtons() {
  // TODO: Implement when button hardware is added
  // Example implementation:
  /*
  uint8_t newStableMask = 0;
  
  for (int i = 0; i < Config::BUTTON_COUNT; i++) {
    // Check if all samples in history match
    uint8_t firstSample = g_buttonState.rawHistory[i][0];
    bool allMatch = true;
    for (int j = 1; j < Config::DEBOUNCE_SAMPLES_REQUIRED; j++) {
      if (g_buttonState.rawHistory[i][j] != firstSample) {
        allMatch = false;
        break;
      }
    }
    
    // If stable, update stable mask
    if (allMatch && firstSample == 1) {
      SET_BIT(newStableMask, i);
    }
  }
  
  // Compute edge mask (transitions since last stable state)
  g_buttonState.edgeMask = (newStableMask ^ g_buttonState.stableMask) & newStableMask;
  g_buttonState.stableMask = newStableMask;
  */
  
  // STUB: Clear masks until hardware exists
  g_buttonState.stableMask = 0;
  g_buttonState.edgeMask = 0;
  
  // Advance history index
  g_buttonState.historyIndex = (g_buttonState.historyIndex + 1) % Config::DEBOUNCE_SAMPLES_REQUIRED;
}

// ======================================================================
// SECTION 9: MODULE C - IR ACQUISITION + EDGE DETECTION
// ======================================================================

// ----- readIRSensorsAndDerive -----
// Read all 4 IR sensors from ADS1115, convert to volts, and derive edge metrics
// Call every 10 ms (IR_PERIOD_MS)
//
// WCET: ~1.5 ms (ADS1115 read time at 860 SPS)
//
// Parameters:
//   - snapshot: SensorSnapshot to populate with IR data and derived metrics
//
void readIRSensorsAndDerive(SensorSnapshot& snapshot) {
  // Read ADS1115 channels (4 differential reads)
  for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
    snapshot.irRaw[i] = g_ads.readADC_SingleEnded(i);
    
    // Convert to volts: raw * 0.125 mV/bit * 0.001 V/mV
    float volts = snapshot.irRaw[i] * Config::ADS1115_GAIN_MULTIPLIER * Config::MILLIVOLTS_TO_VOLTS;
    snapshot.irVolts[i] = clampf(volts, 0.0f, 4.096f);
  }
  
  // Update IR timestamp
  snapshot.irTimestamp = millis();
  
  // Snapshot current thresholds from CommandBlock (thread-safe read)
  DMB();
  for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
    snapshot.thresholds[i] = g_commandBlock.thresholds[i];
  }
  DMB();
  
  // Derive edge detection metrics
  bool sensorOverThreshold[Config::IR_SENSOR_COUNT];
  uint8_t dangerCount = 0;
  
  for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
    sensorOverThreshold[i] = (snapshot.irVolts[i] > snapshot.thresholds[i]);
    if (sensorOverThreshold[i]) {
      dangerCount++;
    }
  }
  
  snapshot.dangerLevel = dangerCount;
  snapshot.edgeDetected = (dangerCount > 0);
  
  // Determine edge direction with priority logic:
  // Priority: FRONT/BACK > diagonals > single-sensor L/R
  bool front_left = sensorOverThreshold[0];
  bool front_right = sensorOverThreshold[1];
  bool back_left = sensorOverThreshold[2];
  bool back_right = sensorOverThreshold[3];
  
  if (front_left && front_right) {
    snapshot.edgeDir = EdgeDirection::FRONT;
  } else if (back_left && back_right) {
    snapshot.edgeDir = EdgeDirection::BACK;
  } else if (front_left && back_left) {
    snapshot.edgeDir = EdgeDirection::LEFT;
  } else if (front_right && back_right) {
    snapshot.edgeDir = EdgeDirection::RIGHT;
  } else if (front_left) {
    snapshot.edgeDir = EdgeDirection::FRONT_LEFT;
  } else if (front_right) {
    snapshot.edgeDir = EdgeDirection::FRONT_RIGHT;
  } else if (back_left) {
    snapshot.edgeDir = EdgeDirection::BACK_LEFT;
  } else if (back_right) {
    snapshot.edgeDir = EdgeDirection::BACK_RIGHT;
  } else {
    snapshot.edgeDir = EdgeDirection::SAFE;
  }
}

// ======================================================================
// SECTION 10: MODULE D - JSON TELEMETRY BUILDER
// ======================================================================

// ----- buildTelemetryJSON -----
// Construct JSON telemetry string from sensor snapshot
// Uses static buffer and snprintf (no dynamic allocation)
//
// WCET: ~5 ms (string formatting overhead)
//
// Parameters:
//   - snapshot: SensorSnapshot containing all sensor data
//
// JSON Format (short keys to minimize bandwidth):
// {
//   "seq": 12345,
//   "tIr": 98765,
//   "tTof": 98760,
//   "irV": [1.23, 2.34, 0.56, 1.78],
//   "irR": [1234, 2345, 567, 1789],
//   "th": [2.5, 2.5, 2.5, 2.5],
//   "dLvl": 2,
//   "eDir": 1,
//   "edg": true,
//   "tofD": [450, 1200, 800],
//   "tofV": 7,
//   "opp": 5,
//   "btn": 0,
//   "btnE": 0,
//   "st": 0
// }
//
void buildTelemetryJSON(const SensorSnapshot& snapshot) {
  char* buf = g_telemetryBuffer;
  int offset = 0;
  
  // Start JSON object
  offset += snprintf(buf + offset, Config::TELEMETRY_BUFFER_SIZE - offset, "{");
  
  // Sequence and timestamps
  offset += snprintf(buf + offset, Config::TELEMETRY_BUFFER_SIZE - offset,
    "\"seq\":%lu,\"tIr\":%lu,\"tTof\":%lu,",
    snapshot.sequence, snapshot.irTimestamp, snapshot.tofTimestamp);
  
  // IR voltages array
  offset += snprintf(buf + offset, Config::TELEMETRY_BUFFER_SIZE - offset,
    "\"irV\":[%.3f,%.3f,%.3f,%.3f],",
    snapshot.irVolts[0], snapshot.irVolts[1], snapshot.irVolts[2], snapshot.irVolts[3]);
  
  // IR raw values array
  offset += snprintf(buf + offset, Config::TELEMETRY_BUFFER_SIZE - offset,
    "\"irR\":[%d,%d,%d,%d],",
    snapshot.irRaw[0], snapshot.irRaw[1], snapshot.irRaw[2], snapshot.irRaw[3]);
  
  // Thresholds array
  offset += snprintf(buf + offset, Config::TELEMETRY_BUFFER_SIZE - offset,
    "\"th\":[%.2f,%.2f,%.2f,%.2f],",
    snapshot.thresholds[0], snapshot.thresholds[1], snapshot.thresholds[2], snapshot.thresholds[3]);
  
  // Danger level, edge direction, edge detected
  offset += snprintf(buf + offset, Config::TELEMETRY_BUFFER_SIZE - offset,
    "\"dLvl\":%u,\"eDir\":%u,\"edg\":%s,",
    snapshot.dangerLevel, static_cast<uint8_t>(snapshot.edgeDir),
    snapshot.edgeDetected ? "true" : "false");
  
  // ToF distances array
  offset += snprintf(buf + offset, Config::TELEMETRY_BUFFER_SIZE - offset,
    "\"tofD\":[%u,%u,%u],",
    snapshot.tofDist[0], snapshot.tofDist[1], snapshot.tofDist[2]);
  
  // ToF valid mask, opponent direction mask
  offset += snprintf(buf + offset, Config::TELEMETRY_BUFFER_SIZE - offset,
    "\"tofV\":%u,\"opp\":%u,",
    snapshot.tofValidMask, snapshot.opponentDirMask);
  
  // Button states
  offset += snprintf(buf + offset, Config::TELEMETRY_BUFFER_SIZE - offset,
    "\"btn\":%u,\"btnE\":%u,",
    snapshot.buttonsStableMask, snapshot.buttonsEdgeMask);
  
  // Status flags
  offset += snprintf(buf + offset, Config::TELEMETRY_BUFFER_SIZE - offset,
    "\"st\":%u",
    snapshot.statusFlags);
  
  // Close JSON object and append newline
  offset += snprintf(buf + offset, Config::TELEMETRY_BUFFER_SIZE - offset, "}\n");
  
  // Ensure null termination (safety)
  g_telemetryBuffer[Config::TELEMETRY_BUFFER_SIZE - 1] = '\0';
}

// ======================================================================
// SECTION 11: MODULE E - CORE1 INTEGRATED SCHEDULER
// ======================================================================

// ----- Core1 Task State -----
struct Core1TaskState {
  uint32_t nextIR;
  uint32_t nextButtonSample;
  uint32_t nextDebounce;
  uint32_t nextToFRight;
  uint32_t nextToFFront;
  uint32_t nextToFLeft;
  uint32_t nextSnapshot;
  uint32_t nextTelemetry;
  uint32_t nextStats;
  
  uint32_t localSequence;  // Monotonic sequence counter
  uint32_t lastCommandSeq;  // Last processed command sequence
  
  SensorSnapshot draftSnapshot;  // Working snapshot (published periodically)
  
  Core1TaskState() : 
    nextIR(0), nextButtonSample(0), nextDebounce(0),
    nextToFRight(0), nextToFFront(0), nextToFLeft(0),
    nextSnapshot(0), nextTelemetry(0), nextStats(0),
    localSequence(0), lastCommandSeq(0) {}
};

Core1TaskState g_core1State;

// ----- pollCommands (Core1) -----
// Check CommandBlock for new commands from Core0
// Apply motor commands and threshold updates if sequence changed
//
// WCET: <50 µs if no change, <200 µs if applying commands
//
void pollCommands() {
  DMB();
  uint32_t currentSeq = g_commandBlock.seq;
  
  if (currentSeq == g_core1State.lastCommandSeq) {
    return;  // No new commands
  }
  
  // New command detected - apply threshold updates
  uint8_t mask = g_commandBlock.thresholdMask;
  for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
    if (TEST_BIT(mask, i)) {
      float newThreshold = g_commandBlock.thresholds[i];
      // Validate threshold range
      if (newThreshold >= Config::IR_EDGE_THRESHOLD_MIN &&
          newThreshold <= Config::IR_EDGE_THRESHOLD_MAX) {
        // Update local threshold (will be copied to snapshot on next IR read)
        g_commandBlock.thresholds[i] = newThreshold;
      }
    }
  }
  
  // TODO: Apply motor commands when motor driver is implemented
  // int16_t left = g_commandBlock.motorLeft;
  // int16_t right = g_commandBlock.motorRight;
  // applyMotorCommands(left, right);
  
  g_core1State.lastCommandSeq = currentSeq;
  DMB();
}

// ----- initCore1 -----
// Initialize Core1 peripherals and task scheduler
// Call once from setup1()
//
void initCore1() {
  Serial.println("=== Core1 Initialization ===");
  
  // Initialize I2C buses
  Wire.begin();  // Wire for ADS1115 (default pins)
  Wire.setClock(400000);  // 400 kHz fast mode
  
  Wire1.setSDA(Config::OLED_SDA_PIN);
  Wire1.setSCL(Config::OLED_SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);
  
  // Initialize Wire1 mutex
  mutex_init(&g_wire1Mutex);
  
  // Initialize ADS1115
  if (!g_ads.begin(Config::ADS1115_ADDRESS, &Wire)) {
    Serial.println("ERROR: ADS1115 init failed!");
    while (1) delay(100);  // Halt on critical failure
  }
  g_ads.setGain(GAIN_ONE);  // ±4.096V range
  g_ads.setDataRate(RATE_ADS1115_860SPS);
  Serial.println("ADS1115: OK");
  
  // Initialize ToF sensors (continuous mode with staggering)
  if (!initToFSensors()) {
    Serial.println("WARNING: ToF sensor initialization incomplete");
    // Continue anyway - some sensors may be functional
  }
  
  // Initialize button pins (stub)
  // TODO: Uncomment when hardware is available
  // pinMode(Config::BUTTON_PIN_START, INPUT_PULLUP);
  // pinMode(Config::BUTTON_PIN_STOP, INPUT_PULLUP);
  
  // Initialize task timing (stagger ToF polls)
  uint32_t now = millis();
  g_core1State.nextIR = now;
  g_core1State.nextButtonSample = now;
  g_core1State.nextDebounce = now + Config::BUTTON_DEBOUNCE_MS;
  g_core1State.nextToFRight = now;  // Start immediately
  g_core1State.nextToFFront = now + Config::TOF_STAGGER_OFFSET_MS;  // +7ms
  g_core1State.nextToFLeft = now + (2 * Config::TOF_STAGGER_OFFSET_MS);  // +14ms
  g_core1State.nextSnapshot = now + Config::SNAPSHOT_PERIOD_MS;
  g_core1State.nextTelemetry = now + Config::TELEMETRY_PERIOD_MS;
  g_core1State.nextStats = now + Config::STATS_PERIOD_MS;
  
  // Initialize draft snapshot
  g_core1State.draftSnapshot = SensorSnapshot();
  g_core1State.draftSnapshot.captureMillis = now;
  
  // Copy default thresholds to CommandBlock
  for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
    g_commandBlock.thresholds[i] = Config::IR_EDGE_THRESHOLD_DEFAULT;
  }
  
  g_core1Ready = true;
  Serial.println("Core1: Ready");
}

// ----- loopCore1 -----
// Core1 main loop - executes tasks based on millis() scheduler
// NO delay() calls - all timing via nextDue timestamps
//
void loopCore1() {
  uint32_t now = millis();
  
  // --- Task: Poll Commands (every loop) ---
  pollCommands();
  
  // --- Task: IR Acquisition (every 10 ms) ---
  if (now >= g_core1State.nextIR) {
    readIRSensorsAndDerive(g_core1State.draftSnapshot);
    g_core1State.nextIR += Config::IR_PERIOD_MS;
  }
  
  // --- Task: Button Sampling (every 5 ms) ---
  if (now >= g_core1State.nextButtonSample) {
    sampleButtonsRaw();
    g_core1State.nextButtonSample += Config::BUTTON_SAMPLE_MS;
  }
  
  // --- Task: Button Debouncing (every 20 ms) ---
  if (now >= g_core1State.nextDebounce) {
    debounceButtons();
    // Copy button state to draft snapshot
    g_core1State.draftSnapshot.buttonsStableMask = g_buttonState.stableMask;
    g_core1State.draftSnapshot.buttonsEdgeMask = g_buttonState.edgeMask;
    g_buttonState.edgeMask = 0;  // Clear edges after copying
    g_core1State.nextDebounce += Config::BUTTON_DEBOUNCE_MS;
  }
  
  // --- Task: ToF Polling (staggered 20 ms periods) ---
  if (now >= g_core1State.nextToFRight) {
    pollOneToFSensor(0, g_core1State.draftSnapshot);  // RIGHT
    g_core1State.nextToFRight += Config::TOF_PERIOD_MS;
  }
  
  if (now >= g_core1State.nextToFFront) {
    pollOneToFSensor(1, g_core1State.draftSnapshot);  // FRONT
    g_core1State.nextToFFront += Config::TOF_PERIOD_MS;
  }
  
  if (now >= g_core1State.nextToFLeft) {
    pollOneToFSensor(2, g_core1State.draftSnapshot);  // LEFT
    g_core1State.nextToFLeft += Config::TOF_PERIOD_MS;
  }
  
  // --- Task: Snapshot Publish (every 10 ms) ---
  if (now >= g_core1State.nextSnapshot) {
    g_core1State.localSequence++;
    g_core1State.draftSnapshot.sequence = g_core1State.localSequence;
    g_core1State.draftSnapshot.captureMillis = now;
    
    // Check data freshness and set status flags
    if (!isTimestampFresh(g_core1State.draftSnapshot.irTimestamp, Config::IR_STALE_MS)) {
      SET_BIT(g_core1State.draftSnapshot.statusFlags, Config::STATUS_IR_STALE);
    } else {
      CLEAR_BIT(g_core1State.draftSnapshot.statusFlags, Config::STATUS_IR_STALE);
    }
    
    if (!isTimestampFresh(g_core1State.draftSnapshot.tofTimestamp, Config::TOF_STALE_MS)) {
      SET_BIT(g_core1State.draftSnapshot.statusFlags, Config::STATUS_TOF_STALE);
    } else {
      CLEAR_BIT(g_core1State.draftSnapshot.statusFlags, Config::STATUS_TOF_STALE);
    }
    
    // Publish snapshot to Core0
    publishSensorSnapshot(g_core1State.draftSnapshot);
    
    g_core1State.nextSnapshot += Config::SNAPSHOT_PERIOD_MS;
  }
  
  // --- Task: Telemetry JSON (every 50 ms) ---
  if (now >= g_core1State.nextTelemetry) {
    buildTelemetryJSON(g_core1State.draftSnapshot);
    
    // TODO: Send telemetry via WiFi TCP (stub for now)
    // Serial.print(g_telemetryBuffer);  // Uncomment for debug
    
    g_core1State.nextTelemetry += Config::TELEMETRY_PERIOD_MS;
  }
  
  // --- Task: Statistics (every 1000 ms) ---
  if (now >= g_core1State.nextStats) {
    float loopFreq = g_core1LoopCount * 1000.0f / now;
    Serial.print("Core1: ");
    Serial.print(loopFreq, 1);
    Serial.print(" Hz | Seq: ");
    Serial.print(g_core1State.localSequence);
    Serial.print(" | ToF: ");
    Serial.print(g_tofInitialized[0] ? "R" : "-");
    Serial.print(g_tofInitialized[1] ? "F" : "-");
    Serial.println(g_tofInitialized[2] ? "L" : "-");
    
    g_core1State.nextStats += Config::STATS_PERIOD_MS;
  }
  
  // Increment loop counter
  g_core1LoopCount++;
}

// ======================================================================
// SECTION 12: CORE0 SUPPORT - STATE MACHINE & COMMAND GENERATION
// ======================================================================

// ----- Core0 State Machine Context -----
struct Core0Context {
  SensorSnapshot latestSnapshot;
  uint32_t lastSnapshotSeq;
  bool snapshotStale;
  
  Core0Context() : lastSnapshotSeq(0), snapshotStale(false) {}
};

Core0Context g_core0Context;

// ----- fetchAndCheckStaleness (Core0) -----
// Fetch latest snapshot from Core1 and check for staleness
// Triggers safety fallback if data is too old (>40ms)
//
// WCET: <50 µs
//
void fetchAndCheckStaleness() {
  if (!fetchLatestSnapshot(g_core0Context.latestSnapshot)) {
    // Fetch failed (rare) - keep using old snapshot
    g_core0Context.snapshotStale = true;
    return;
  }
  
  // Check if snapshot sequence changed
  if (g_core0Context.latestSnapshot.sequence == g_core0Context.lastSnapshotSeq) {
    // No new data - check age
    uint32_t age = millis() - g_core0Context.latestSnapshot.captureMillis;
    if (age > Config::SNAPSHOT_STALE_MS) {
      g_core0Context.snapshotStale = true;
    }
  } else {
    // New data received
    g_core0Context.lastSnapshotSeq = g_core0Context.latestSnapshot.sequence;
    g_core0Context.snapshotStale = false;
  }
}

// ----- stateMachineLogic (Core0) -----
// Example state machine stub using sensor snapshot data
// Replace with actual tactical logic for Bottle Sumo competition
//
// WCET: <100 µs
//
void stateMachineLogic() {
  const SensorSnapshot& snap = g_core0Context.latestSnapshot;
  
  // Safety fallback: if data is stale, stop motors
  if (g_core0Context.snapshotStale) {
    updateCommands(0, 0);  // Stop motors
    return;
  }
  
  // Example tactical logic (STUB - replace with actual strategy):
  
  // Priority 1: Edge avoidance
  if (snap.edgeDetected) {
    // Reverse away from edge
    switch (snap.edgeDir) {
      case EdgeDirection::FRONT:
        updateCommands(-200, -200);  // Reverse
        break;
      case EdgeDirection::BACK:
        updateCommands(200, 200);    // Forward
        break;
      case EdgeDirection::LEFT:
        updateCommands(150, -150);   // Spin right
        break;
      case EdgeDirection::RIGHT:
        updateCommands(-150, 150);   // Spin left
        break;
      default:
        updateCommands(-150, -150);  // General retreat
        break;
    }
    return;
  }
  
  // Priority 2: Opponent attack
  if (snap.opponentDirMask != 0) {
    // Attack opponent
    bool frontOpponent = TEST_BIT(snap.opponentDirMask, 1);  // FRONT sensor
    bool rightOpponent = TEST_BIT(snap.opponentDirMask, 0);  // RIGHT sensor
    bool leftOpponent = TEST_BIT(snap.opponentDirMask, 2);   // LEFT sensor
    
    if (frontOpponent) {
      updateCommands(255, 255);  // Full speed forward
    } else if (rightOpponent) {
      updateCommands(200, 100);  // Turn right toward opponent
    } else if (leftOpponent) {
      updateCommands(100, 200);  // Turn left toward opponent
    }
    return;
  }
  
  // Priority 3: Search pattern (no edge, no opponent)
  updateCommands(150, 120);  // Slow circular search
}

// ----- updateCommands (Core0) -----
// Write new motor commands to CommandBlock for Core1 to consume
// Increments sequence number and uses memory barrier
//
// WCET: <10 µs
//
// Parameters:
//   - left: Left motor PWM (-255 to +255)
//   - right: Right motor PWM (-255 to +255)
//
void updateCommands(int16_t left, int16_t right) {
  DMB();
  g_commandBlock.motorLeft = left;
  g_commandBlock.motorRight = right;
  g_commandBlock.seq++;
  DMB();
}

// ----- updateThresholds (Core0) -----
// Update edge detection thresholds via CommandBlock
// Used by TCP command handler
//
// Parameters:
//   - thresholds: Array of 4 threshold values (volts)
//   - mask: Bitmask indicating which thresholds to update
//
void updateThresholds(float thresholds[Config::IR_SENSOR_COUNT], uint8_t mask) {
  DMB();
  for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
    if (TEST_BIT(mask, i)) {
      g_commandBlock.thresholds[i] = thresholds[i];
    }
  }
  g_commandBlock.thresholdMask = mask;
  g_commandBlock.seq++;
  DMB();
}

// ======================================================================
// SECTION 13: ARDUINO SETUP & LOOP (CORE0)
// ======================================================================

// ----- setup (Core0) -----
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait up to 3s for serial
  
  Serial.println("\n\n======================================");
  Serial.println("RP2040 Dual-Core Bottle Sumo Firmware");
  Serial.println("Version: 1.0.0");
  Serial.println("======================================\n");
  
  Serial.println("Core0: Initializing...");
  
  // Initialize Core0 context
  g_core0Context = Core0Context();
  
  // Initialize command block with default thresholds
  for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
    g_commandBlock.thresholds[i] = Config::IR_EDGE_THRESHOLD_DEFAULT;
  }
  
  // TODO: Initialize WiFi and TCP server (stub)
  // initWiFiAP();
  // initTCPServer();
  
  Serial.println("Core0: Ready");
  Serial.println("Waiting for Core1 initialization...");
}

// ----- loop (Core0) -----
void loop() {
  // Wait for Core1 to complete initialization
  if (!g_core1Ready) {
    delay(10);
    return;
  }
  
  // Fetch latest sensor snapshot from Core1
  fetchAndCheckStaleness();
  
  // Execute state machine logic
  stateMachineLogic();
  
  // TODO: Handle WiFi/TCP connections and commands (stub)
  // handleTCPClients();
  
  // TODO: Update OLED display (stub - use Wire1 mutex)
  // updateOLED();
  
  // Increment loop counter
  g_core0LoopCount++;
  
  // Maintain ~100 Hz loop frequency
  delay(10);
}

// ======================================================================
// SECTION 14: ARDUINO SETUP1 & LOOP1 (CORE1)
// ======================================================================

// ----- setup1 (Core1) -----
void setup1() {
  // Small delay to let Core0 initialize Serial
  delay(100);
  
  initCore1();
}

// ----- loop1 (Core1) -----
void loop1() {
  loopCore1();
}

// ======================================================================
// END OF FIRMWARE
// ======================================================================
