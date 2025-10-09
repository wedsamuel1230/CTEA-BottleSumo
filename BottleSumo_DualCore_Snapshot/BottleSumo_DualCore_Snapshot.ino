/*
 * Dual-Core Bottle Sumo Robot - Lockless Snapshot Architecture
 * Using Raspberry Pi Pico W dual-core with lockless double-buffer pattern
 * 
 * Architecture:
 * Core 1 (Sensor Core): High-speed sensor acquisition (IR + ToF + buttons)
 *                       Preprocessing and publishing snapshots at ~100Hz
 * Core 0 (Logic Core):  State machine, decision making, motor control
 *                       Consumes latest snapshots locklessly
 * 
 * Communication Pattern:
 * - Core 1 → Core 0: Lockless double-buffer with atomic index swap
 * - Core 0 → Core 1: Command block with sequence number
 * 
 * Benefits:
 * - No mutex contention in hot path
 * - Low latency (<10-15ms)
 * - Derived metrics computed once on Core 1
 * - Constant-time publish/consume operations
 * 
 * Hardware:
 * - Raspberry Pi Pico W
 * - ADS1115 ADC (I2C, Wire)
 * - 4x QRE1113 IR sensors (edge detection)
 * - 3x VL53L1X ToF sensors (opponent detection)
 * - Optional: Buttons for mode control
 * - Optional: OLED display (I2C, Wire1)
 * 
 * Author: CTEA-BottleSumo Project
 * Date: 2025
 */

#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <VL53L0X.h>

// Include our snapshot architecture components
#include "src/sensor_shared.cpp"
#include "src/core1_publish.cpp"
#include "src/core0_consume.cpp"

// ========== Configuration ==========

namespace Config {
  // Hardware configuration
  constexpr uint8_t ADS1115_I2C_ADDRESS = 0x48;
  constexpr uint8_t IR_SENSOR_COUNT = 4;
  constexpr uint8_t TOF_SENSOR_COUNT = 3;
  
  // I2C pins
  constexpr uint8_t I2C0_SDA_PIN = 4;  // GP4 for Wire (ADS1115)
  constexpr uint8_t I2C0_SCL_PIN = 5;  // GP5 for Wire
  constexpr uint8_t I2C1_SDA_PIN = 26; // GP26 for Wire1 (ToF sensors)
  constexpr uint8_t I2C1_SCL_PIN = 27; // GP27 for Wire1
  
  // ToF sensor XSHUT pins
  constexpr uint8_t TOF_XSHUT_RIGHT = 13;  // GP13
  constexpr uint8_t TOF_XSHUT_FRONT = 12;  // GP12
  constexpr uint8_t TOF_XSHUT_LEFT = 11;   // GP11
  
  // ToF sensor I2C addresses (after reassignment)
  constexpr uint8_t TOF_ADDR_RIGHT = 0x30;
  constexpr uint8_t TOF_ADDR_FRONT = 0x32;
  constexpr uint8_t TOF_ADDR_LEFT = 0x34;
  
  // Button pins (optional)
  constexpr uint8_t BUTTON_PINS[4] = {3, 4, 5, 6}; // GP3-GP6
  constexpr uint8_t BUTTON_COUNT = 4;
  
  // Timing configuration
  constexpr unsigned long CORE1_PUBLISH_INTERVAL_MS = 10;  // 100 Hz
  constexpr unsigned long CORE0_LOOP_DELAY_MS = 10;        // 100 Hz
  constexpr unsigned long TOF_READ_INTERVAL_MS = 100;      // 10 Hz
  constexpr unsigned long BUTTON_DEBOUNCE_MS = 20;         // 20ms debounce
  
  // Thresholds
  constexpr float EDGE_THRESHOLD_VOLTS = 2.5f;
  constexpr uint16_t TOF_DETECTION_THRESHOLD_MM = 1600;    // 160cm
  constexpr uint16_t TOF_MAX_VALID_RANGE_MM = 1500;
  constexpr uint16_t TOF_MIN_VALID_RANGE_MM = 30;
  
  // I2C speeds
  constexpr uint32_t I2C_FAST_MODE_HZ = 400000;
}

// ========== Global Objects ==========

Adafruit_ADS1115 ads;
VL53L1X tofSensors[Config::TOF_SENSOR_COUNT];

// ToF sensor initialization status
bool tofInitialized[Config::TOF_SENSOR_COUNT] = {false, false, false};

// Runtime thresholds (can be updated via command channel)
float runtimeThresholds[Config::IR_SENSOR_COUNT] = {
  Config::EDGE_THRESHOLD_VOLTS,
  Config::EDGE_THRESHOLD_VOLTS,
  Config::EDGE_THRESHOLD_VOLTS,
  Config::EDGE_THRESHOLD_VOLTS
};

// Button state tracking
struct ButtonState {
  uint8_t stableMask;
  uint8_t edgeMask;
  uint8_t lastRaw;
  unsigned long lastChangeTime;
};

ButtonState buttonState = {0, 0, 0, 0};

// Core status
volatile bool core1Active = false;
volatile unsigned long core0LoopCount = 0;
volatile unsigned long core1LoopCount = 0;

// ========== Forward Declarations ==========

void updateMotorAndThresholds(int16_t left, int16_t right, const float* newThresh, uint8_t mask, uint16_t flags);
void pollCommands();
void applyMotorOutputs(int16_t left, int16_t right);

// ========== Core 1: Sensor Acquisition Functions ==========

// Initialize ADS1115 ADC
bool initADS1115() {
  Serial.println("Initializing ADS1115...");
  ads.setGain(GAIN_ONE);  // ±4.096V range
  ads.setDataRate(RATE_ADS1115_860SPS);  // Max sample rate
  
  if (!ads.begin(Config::ADS1115_I2C_ADDRESS, &Wire)) {
    Serial.println("❌ Failed to initialize ADS1115");
    return false;
  }
  
  Serial.println("✓ ADS1115 initialized");
  return true;
}

// Initialize ToF sensors with address reassignment
bool initToFSensors() {
  Serial.println("Initializing ToF sensors...");
  
  // Reset all sensors
  pinMode(Config::TOF_XSHUT_RIGHT, OUTPUT);
  pinMode(Config::TOF_XSHUT_FRONT, OUTPUT);
  pinMode(Config::TOF_XSHUT_LEFT, OUTPUT);
  digitalWrite(Config::TOF_XSHUT_RIGHT, LOW);
  digitalWrite(Config::TOF_XSHUT_FRONT, LOW);
  digitalWrite(Config::TOF_XSHUT_LEFT, LOW);
  delay(50);
  
  bool allOk = true;
  
  // Initialize RIGHT sensor
  digitalWrite(Config::TOF_XSHUT_RIGHT, HIGH);
  delay(50);
  tofSensors[0].setBus(&Wire1);
  tofSensors[0].setAddress(0x29);  // Default address
  if (tofSensors[0].init()) {
    tofSensors[0].setDistanceMode(VL53L1X::Short);
    tofSensors[0].setMeasurementTimingBudget(50000);  // 50ms
    tofSensors[0].startContinuous(50);
    tofSensors[0].setAddress(Config::TOF_ADDR_RIGHT);
    tofInitialized[0] = true;
    Serial.println("✓ RIGHT ToF initialized");
  } else {
    Serial.println("❌ RIGHT ToF failed");
    allOk = false;
  }
  
  // Initialize FRONT sensor
  digitalWrite(Config::TOF_XSHUT_FRONT, HIGH);
  delay(50);
  tofSensors[1].setBus(&Wire1);
  tofSensors[1].setAddress(0x29);
  if (tofSensors[1].init()) {
    tofSensors[1].setDistanceMode(VL53L1X::Short);
    tofSensors[1].setMeasurementTimingBudget(50000);
    tofSensors[1].startContinuous(50);
    tofSensors[1].setAddress(Config::TOF_ADDR_FRONT);
    tofInitialized[1] = true;
    Serial.println("✓ FRONT ToF initialized");
  } else {
    Serial.println("❌ FRONT ToF failed");
    allOk = false;
  }
  
  // Initialize LEFT sensor
  digitalWrite(Config::TOF_XSHUT_LEFT, HIGH);
  delay(50);
  tofSensors[2].setBus(&Wire1);
  tofSensors[2].setAddress(0x29);
  if (tofSensors[2].init()) {
    tofSensors[2].setDistanceMode(VL53L1X::Short);
    tofSensors[2].setMeasurementTimingBudget(50000);
    tofSensors[2].startContinuous(50);
    tofSensors[2].setAddress(Config::TOF_ADDR_LEFT);
    tofInitialized[2] = true;
    Serial.println("✓ LEFT ToF initialized");
  } else {
    Serial.println("❌ LEFT ToF failed");
    allOk = false;
  }
  
  return allOk;
}

// Read IR sensors and compute voltages
void readIRSensors(int16_t* rawValues, float* voltages) {
  for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
    rawValues[i] = ads.readADC_SingleEnded(i);
    voltages[i] = ads.computeVolts(rawValues[i]);
  }
}

// Read ToF sensors
void readToFSensors(uint16_t* distances, uint8_t* validMask) {
  *validMask = 0;
  
  for (int i = 0; i < Config::TOF_SENSOR_COUNT; i++) {
    if (tofInitialized[i]) {
      uint16_t dist = tofSensors[i].read();
      if (dist >= Config::TOF_MIN_VALID_RANGE_MM && 
          dist <= Config::TOF_MAX_VALID_RANGE_MM) {
        distances[i] = dist;
        *validMask |= (1 << i);
      } else {
        distances[i] = 0;
      }
    } else {
      distances[i] = 0;
    }
  }
}

// Button debouncing and edge detection
void updateButtons(uint8_t* stableMask, uint8_t* edgeMask) {
  unsigned long now = millis();
  uint8_t rawMask = 0;
  
  // Read all buttons
  for (int i = 0; i < Config::BUTTON_COUNT; i++) {
    if (digitalRead(Config::BUTTON_PINS[i]) == LOW) {  // Active low
      rawMask |= (1 << i);
    }
  }
  
  // Debounce logic
  if (rawMask != buttonState.lastRaw) {
    buttonState.lastRaw = rawMask;
    buttonState.lastChangeTime = now;
    *edgeMask = 0;  // No edge during transition
  } else if ((now - buttonState.lastChangeTime) >= Config::BUTTON_DEBOUNCE_MS) {
    // Stable for debounce period
    uint8_t newStable = rawMask;
    *edgeMask = newStable ^ buttonState.stableMask;
    buttonState.stableMask = newStable;
  } else {
    *edgeMask = 0;
  }
  
  *stableMask = buttonState.stableMask;
}

// Compute edge direction from IR voltages
EdgeDirection computeEdgeDirection(const float* voltages, const float* thresholds) {
  bool frontLeft = voltages[0] > thresholds[0];
  bool frontRight = voltages[1] > thresholds[1];
  bool backLeft = voltages[2] > thresholds[2];
  bool backRight = voltages[3] > thresholds[3];
  
  if (frontLeft && frontRight) return EDGE_FRONT;
  if (backLeft && backRight) return EDGE_BACK;
  if (frontLeft || backLeft) return EDGE_LEFT;
  if (frontRight || backRight) return EDGE_RIGHT;
  if (frontLeft) return EDGE_FRONT_LEFT;
  if (frontRight) return EDGE_FRONT_RIGHT;
  if (backLeft) return EDGE_BACK_LEFT;
  if (backRight) return EDGE_BACK_RIGHT;
  return EDGE_SAFE;
}

// Compute danger level (0-4)
uint8_t computeDangerLevel(const float* voltages, const float* thresholds) {
  uint8_t count = 0;
  for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
    if (voltages[i] > thresholds[i]) {
      count++;
    }
  }
  return count;
}

// Compute opponent direction from ToF readings
uint8_t computeOpponentDirection(const uint16_t* distances, uint8_t validMask, uint16_t threshold) {
  uint8_t dirMask = 0;
  
  for (int i = 0; i < Config::TOF_SENSOR_COUNT; i++) {
    if ((validMask & (1 << i)) && distances[i] < threshold) {
      dirMask |= (1 << i);
    }
  }
  
  return dirMask;
}

// ========== Core 1: Main Loop ==========

void setup1() {
  delay(100);  // Let Core 0 initialize serial
  
  Serial.println("Core 1: Sensor acquisition core starting...");
  
  // Initialize I2C buses
  Wire.setSDA(Config::I2C0_SDA_PIN);
  Wire.setSCL(Config::I2C0_SCL_PIN);
  Wire.begin();
  Wire.setClock(Config::I2C_FAST_MODE_HZ);
  
  Wire1.setSDA(Config::I2C1_SDA_PIN);
  Wire1.setSCL(Config::I2C1_SCL_PIN);
  Wire1.begin();
  Wire1.setClock(Config::I2C_FAST_MODE_HZ);
  
  // Initialize sensors
  if (!initADS1115()) {
    Serial.println("❌ Core 1: ADS1115 initialization failed");
    while (1) delay(1000);
  }
  
  initToFSensors();  // ToF failures are not fatal
  
  // Initialize button pins
  for (int i = 0; i < Config::BUTTON_COUNT; i++) {
    pinMode(Config::BUTTON_PINS[i], INPUT_PULLUP);
  }
  
  core1Active = true;
  Serial.println("✓ Core 1: Ready");
}

void loop1() {
  static unsigned long lastPublish = 0;
  static unsigned long lastToFRead = 0;
  unsigned long now = millis();
  
  core1LoopCount++;
  
  // Read IR sensors continuously (fast, ~860 SPS)
  int16_t irRaw[Config::IR_SENSOR_COUNT];
  float irVolts[Config::IR_SENSOR_COUNT];
  readIRSensors(irRaw, irVolts);
  
  // Read ToF sensors at lower rate (10 Hz)
  static uint16_t tofDist[Config::TOF_SENSOR_COUNT] = {0};
  static uint8_t tofValidMask = 0;
  if ((now - lastToFRead) >= Config::TOF_READ_INTERVAL_MS) {
    readToFSensors(tofDist, &tofValidMask);
    lastToFRead = now;
  }
  
  // Read buttons
  uint8_t buttonStable, buttonEdge;
  updateButtons(&buttonStable, &buttonEdge);
  
  // Publish snapshot at controlled rate (100 Hz)
  if ((now - lastPublish) >= Config::CORE1_PUBLISH_INTERVAL_MS) {
    SensorSnapshot draft;
    
    // Timing
    draft.captureMillis = now;
    draft.tofMillis = lastToFRead;
    
    // IR data
    for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
      draft.irRaw[i] = irRaw[i];
      draft.irVolts[i] = irVolts[i];
      draft.thresholds[i] = runtimeThresholds[i];
    }
    
    // Derived edge info
    draft.edgeDir = computeEdgeDirection(irVolts, runtimeThresholds);
    draft.dangerLevel = computeDangerLevel(irVolts, runtimeThresholds);
    draft.edgeDetected = (draft.dangerLevel > 0) ? 1 : 0;
    
    // ToF data
    for (int i = 0; i < Config::TOF_SENSOR_COUNT; i++) {
      draft.tofDist[i] = tofDist[i];
    }
    draft.tofValidMask = tofValidMask;
    draft.opponentDirMask = computeOpponentDirection(tofDist, tofValidMask, Config::TOF_DETECTION_THRESHOLD_MM);
    
    // Button data
    draft.buttonsStableMask = buttonStable;
    draft.buttonsEdgeMask = buttonEdge;
    
    // Status flags
    draft.statusFlags = 0;
    for (int i = 0; i < Config::TOF_SENSOR_COUNT; i++) {
      if (!tofInitialized[i]) {
        draft.statusFlags |= (1 << i);  // Mark offline sensors
      }
    }
    
    // Publish snapshot (lockless)
    publishSensorSnapshot(draft);
    
    lastPublish = now;
  }
  
  // Poll for commands from Core 0
  pollCommands();
  
  // Minimal delay to yield CPU
  delay(1);
}

// ========== Core 0: Command Functions ==========

void updateMotorAndThresholds(int16_t left, int16_t right, const float* newThresh, uint8_t mask, uint16_t flags) {
  static uint32_t cmdSeq = 0;
  
  g_commandBlock.motorLeft = left;
  g_commandBlock.motorRight = right;
  g_commandBlock.flags = flags;
  
  if (mask) {
    for (int i = 0; i < 4; i++) {
      if (mask & (1 << i)) {
        g_commandBlock.thresholds[i] = newThresh[i];
      }
    }
    g_commandBlock.thresholdMask = mask;
  }
  
  cmdSeq++;
  __asm volatile("" ::: "memory");  // Memory barrier
  g_commandBlock.seq = cmdSeq;
}

void applyMotorOutputs(int16_t left, int16_t right) {
  // TODO: Implement actual motor control
  // This is a placeholder for motor driver logic
  static int16_t lastLeft = 0, lastRight = 0;
  
  if (left != lastLeft || right != lastRight) {
    Serial.printf("Motor: L=%d R=%d\n", left, right);
    lastLeft = left;
    lastRight = right;
  }
}

void pollCommands() {
  static uint32_t lastApplied = 0;
  uint32_t seq = g_commandBlock.seq;
  
  if (seq != lastApplied) {
    // Apply motor setpoints
    int16_t l = g_commandBlock.motorLeft;
    int16_t r = g_commandBlock.motorRight;
    applyMotorOutputs(l, r);
    
    // Apply threshold changes
    uint8_t mask = g_commandBlock.thresholdMask;
    if (mask) {
      for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
        if (mask & (1 << i)) {
          runtimeThresholds[i] = g_commandBlock.thresholds[i];
        }
      }
      g_commandBlock.thresholdMask = 0;  // Clear after apply
    }
    
    // TODO: Handle flags (g_commandBlock.flags)
    
    lastApplied = seq;
  }
}

// ========== Core 0: State Machine ==========

enum RobotState {
  STATE_INIT,
  STATE_SEARCH,
  STATE_ATTACK,
  STATE_RETREAT,
  STATE_EMERGENCY
};

const char* stateNames[] = {
  "INIT",
  "SEARCH",
  "ATTACK",
  "RETREAT",
  "EMERGENCY"
};

RobotState currentState = STATE_INIT;

void runStateMachine(const SensorSnapshot& snap) {
  static unsigned long stateEntryTime = 0;
  RobotState nextState = currentState;
  
  // Emergency override: edge detected with high danger
  if (snap.dangerLevel >= 2) {
    nextState = STATE_EMERGENCY;
  }
  // Edge detected with low danger
  else if (snap.edgeDetected) {
    nextState = STATE_RETREAT;
  }
  // Opponent detected
  else if (snap.opponentDirMask != 0) {
    nextState = STATE_ATTACK;
  }
  // Default: search for opponent
  else {
    nextState = STATE_SEARCH;
  }
  
  // State transition
  if (nextState != currentState) {
    Serial.printf("State: %s -> %s\n", stateNames[currentState], stateNames[nextState]);
    currentState = nextState;
    stateEntryTime = millis();
  }
  
  // Execute state actions
  switch (currentState) {
    case STATE_INIT:
      // Initialization complete, transition to search
      nextState = STATE_SEARCH;
      break;
      
    case STATE_SEARCH:
      // Rotate slowly to find opponent
      updateMotorAndThresholds(50, -50, nullptr, 0, 0);  // Spin in place
      break;
      
    case STATE_ATTACK:
      // Move toward opponent
      updateMotorAndThresholds(100, 100, nullptr, 0, 0);  // Forward
      break;
      
    case STATE_RETREAT:
      // Back away from edge
      {
        int16_t leftMotor = -80;
        int16_t rightMotor = -80;
        
        // Adjust based on edge direction
        switch (snap.edgeDir) {
          case EDGE_LEFT:
            leftMotor = -100;
            rightMotor = -60;
            break;
          case EDGE_RIGHT:
            leftMotor = -60;
            rightMotor = -100;
            break;
          case EDGE_FRONT:
            leftMotor = -100;
            rightMotor = -100;
            break;
          case EDGE_BACK:
            leftMotor = 60;
            rightMotor = 60;
            break;
        }
        
        updateMotorAndThresholds(leftMotor, rightMotor, nullptr, 0, 0);
      }
      break;
      
    case STATE_EMERGENCY:
      // Stop immediately
      updateMotorAndThresholds(0, 0, nullptr, 0, 0);
      break;
  }
}

// ========== Core 0: Main Loop ==========

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("========================================");
  Serial.println("Bottle Sumo - Dual-Core Snapshot Architecture");
  Serial.println("========================================");
  Serial.println("Core 0: State machine and control");
  Serial.println("Core 1: Sensor acquisition");
  Serial.println("");
  
  // Wait for Core 1 to initialize
  Serial.println("Waiting for Core 1 initialization...");
  while (!core1Active) {
    delay(10);
  }
  
  Serial.println("✓ System ready");
  Serial.println("========================================");
}

void loop() {
  static unsigned long lastStatusPrint = 0;
  unsigned long now = millis();
  
  core0LoopCount++;
  
  // Fetch latest snapshot (lockless)
  SensorSnapshot snap;
  bool isNew = fetchLatestSnapshot(snap);
  
  // Check data staleness
  if ((now - snap.captureMillis) > 50) {
    Serial.println("⚠️ Warning: Stale data");
  }
  
  // Run state machine
  runStateMachine(snap);
  
  // Status printing (every 5 seconds)
  if ((now - lastStatusPrint) >= 5000) {
    Serial.println("========================================");
    Serial.printf("Uptime: %lu s\n", now / 1000);
    Serial.printf("Core 0: %.1f Hz | Core 1: %.1f Hz\n", 
                  core0LoopCount * 1000.0 / now,
                  core1LoopCount * 1000.0 / now);
    Serial.printf("Sequence: %lu | Age: %lu ms\n", 
                  snap.sequence, now - snap.captureMillis);
    Serial.printf("State: %s\n", stateNames[currentState]);
    Serial.printf("Edge: %s | Danger: %d/4\n", 
                  snap.edgeDetected ? "YES" : "NO", snap.dangerLevel);
    Serial.printf("IR Volts: [%.2f, %.2f, %.2f, %.2f]\n",
                  snap.irVolts[0], snap.irVolts[1], 
                  snap.irVolts[2], snap.irVolts[3]);
    Serial.printf("ToF Dist: [%d, %d, %d] | Valid: 0x%02X\n",
                  snap.tofDist[0], snap.tofDist[1], snap.tofDist[2],
                  snap.tofValidMask);
    Serial.printf("Buttons: Stable=0x%02X Edge=0x%02X\n",
                  snap.buttonsStableMask, snap.buttonsEdgeMask);
    Serial.println("========================================");
    
    lastStatusPrint = now;
  }
  
  // Rate limiting
  delay(Config::CORE0_LOOP_DELAY_MS);
}
