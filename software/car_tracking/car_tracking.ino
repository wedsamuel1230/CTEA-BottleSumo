/**
 * Car Tracking with Dual-Core Multi-Sensor Directional Control + WiFi Telemetry
 * 
 * This sketch implements autonomous object tracking using a 5-sensor ToF array
 * with RP2040 dual-core architecture and real-time TCP telemetry streaming.
 * 
 * DUAL-CORE ARCHITECTURE:
 *   Core0 (Tracking): Sensor reading + Tracking algorithm → Motor commands + Telemetry
 *   Core1 (Motor+WiFi): Motor control + WiFi AP + TCP telemetry server
 * 
 * Hardware Configuration:
 *   Target: RP2040 Pico W (WiFi-enabled)
 *   
 *   Motors:
 *     Left Motor:  PWM=GP11, DIR=GP12
 *     Right Motor: PWM=GP14, DIR=GP15
 *   
 *   ToF Sensors (VL53L0X):
 *     Index 0: Right 45° (R45) - XSHUT GP8 - I2C 0x30
 *     Index 1: Right 23° (R23) - XSHUT GP7 - I2C 0x31
 *     Index 2: Middle  0° (M0)  - XSHUT GP6 - I2C 0x32
 *     Index 3: Left  23° (L23)  - XSHUT GP5 - I2C 0x33
 *     Index 4: Left  45° (L45)  - XSHUT GP4 - I2C 0x34
 *   
 *   I2C Bus:
 *     Wire1: SDA=GP2, SCL=GP3
 * 
 *   WiFi Telemetry:
 *     AP SSID: PicoW-CarTracker
 *     AP Password: tracking123
 *     AP IP: 192.168.4.1
 *     TCP Port: 8080
 *     Format: JSON streaming (10Hz updates)
 * 
 * Tracking Algorithm:
 *   - Multi-sensor weighted directional tracking
 *   - Calculates left vs right sensor bias
 *   - Applies differential steering to center on target
 *   - Speed scales with distance (closer = slower for safety)
 * 
 * Performance Benefits:
 *   - Core1 motor response time: <20ms (vs ~100ms single-core)
 *   - Core0 can take full time for I2C without blocking motors
 *   - True parallelism: sensors, motors, and WiFi run simultaneously
 *   - Real-time telemetry streaming via TCP (JSON format)
 * 
 * Author: Autonomous Copilot Agent
 * Date: 2025-11-10 (Dual-Core + WiFi Telemetry)
 */

#include <Wire.h>
#include <WiFi.h>
#include <pico/mutex.h>
#include "Car.h"
#include "ToFArray.h"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Motor Pin Assignments
const uint8_t LEFT_MOTOR_PWM = 11;   // GP11
const uint8_t LEFT_MOTOR_DIR = 12;   // GP12
const uint8_t RIGHT_MOTOR_PWM = 14;  // GP14
const uint8_t RIGHT_MOTOR_DIR = 15;  // GP15
const uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz (silent operation)

// ToF Sensor Configuration
const uint8_t TOF_NUM = 5;
const uint8_t TOF_XSHUT_PINS[TOF_NUM] = {8, 7, 6, 5, 4};  // GP8-GP4
const uint8_t TOF_I2C_ADDR[TOF_NUM] = {0x30, 0x31, 0x32, 0x33, 0x34};
const char* TOF_NAMES[TOF_NUM] = {"R45", "R23", "M0", "L23", "L45"};

// I2C Configuration
const uint8_t I2C_SDA = 2;  // GP2
const uint8_t I2C_SCL = 3;  // GP3

// WiFi Configuration (AP Mode)
const char* WIFI_SSID = "PicoW-CarTracker";
const char* WIFI_PASSWORD = "tracking123";
const uint16_t TCP_PORT = 8080;

// ============================================================================
// TRACKING PARAMETERS
// ============================================================================

// Distance thresholds (millimeters)
const uint16_t DETECT_MIN_MM = 50;    // Minimum detection distance
const uint16_t DETECT_MAX_MM = 1000;   // Maximum detection distance
const uint16_t MIDDLE_CLEAR_THRESHOLD = 100;  // Middle sensor: ≥100mm = clear to move forward

// Speed parameters (0-100 range)
const float ROTATION_SPEED = 30.0f;   // Speed for rotating to face object
const float BIAS_DEADZONE = 0.15f;    // Deadzone for "centered" detection (0.0-1.0)

// Timing (milliseconds)
const uint32_t READ_INTERVAL = 100;   // Sensor read interval (10 Hz) - Core0
const uint32_t MOTOR_INTERVAL = 20;   // Motor update interval (50 Hz) - Core1
const uint32_t TELEMETRY_INTERVAL = 100;  // Telemetry broadcast interval (10 Hz) - Core1
const uint32_t LOST_TIMEOUT = 2000;   // Stop if no target for 2 seconds
const uint32_t WATCHDOG_TIMEOUT = 500;  // Stop motors if Core0 unresponsive

// ============================================================================
// INTER-CORE COMMUNICATION
// ============================================================================

/**
 * Shared motor command structure (protected by mutex)
 * Written by Core0 (tracking), read by Core1 (motor control)
 */
struct MotorCommand {
  float leftSpeed;      // -100 to +100
  float rightSpeed;     // -100 to +100
  uint32_t timestamp;   // millis() for watchdog detection
  bool valid;           // Command validity flag
  bool emergencyStop;   // Emergency stop flag
};

/**
 * Shared telemetry data structure (protected by mutex)
 * Written by Core0 (tracking), read by Core1 (WiFi/TCP)
 */
struct TelemetryData {
  uint16_t distance;      // Closest distance in mm
  float bias;             // Direction bias (-1.0 to +1.0)
  float speed;            // Calculated speed
  float leftSpeed;        // Left motor speed
  float rightSpeed;       // Right motor speed
  uint16_t sensors[5];    // Sensor readings [R45, R23, M0, L23, L45]
  uint32_t timestamp;     // millis() timestamp
  bool valid;             // Data validity flag
};

// Shared data (accessed by both cores)
static MotorCommand sharedMotorCmd = {0.0f, 0.0f, 0, false, false};
static TelemetryData sharedTelemetry = {0, 0.0f, 0.0f, 0.0f, 0.0f, {0,0,0,0,0}, 0, false};
static mutex_t motorCmdMutex;
static mutex_t telemetryMutex;

// ============================================================================
// CORE-SPECIFIC OBJECTS
// ============================================================================

// Core0 objects (Tracking)
ToFArray tof(&Wire1, nullptr);
ToFSample tofData[TOF_NUM];
unsigned long lastRead = 0;
unsigned long lastDetection = 0;
bool trackingReady = false;

// Core1 objects (Motor Control + WiFi)
Car car;
WiFiServer tcpServer(TCP_PORT);
WiFiClient connectedClients[4];  // Support up to 4 simultaneous clients
uint8_t clientCount = 0;
unsigned long lastMotorUpdate = 0;
unsigned long lastTelemetryBroadcast = 0;
bool motorReady = false;
bool wifiReady = false;

// ============================================================================
// INTER-CORE COMMUNICATION HELPERS
// ============================================================================

/**
 * Send motor command from Core0 to Core1 (thread-safe)
 */
void sendMotorCommand(float leftSpeed, float rightSpeed, bool emergencyStop = false) {
  mutex_enter_blocking(&motorCmdMutex);
  sharedMotorCmd.leftSpeed = leftSpeed;
  sharedMotorCmd.rightSpeed = rightSpeed;
  sharedMotorCmd.timestamp = millis();
  sharedMotorCmd.valid = true;
  sharedMotorCmd.emergencyStop = emergencyStop;
  mutex_exit(&motorCmdMutex);
}

/**
 * Read motor command on Core1 (thread-safe)
 * Returns false if no valid command or watchdog timeout
 */
bool readMotorCommand(MotorCommand& cmd) {
  mutex_enter_blocking(&motorCmdMutex);
  cmd = sharedMotorCmd;
  mutex_exit(&motorCmdMutex);
  
  // Check watchdog timeout
  if (cmd.valid && (millis() - cmd.timestamp) > WATCHDOG_TIMEOUT) {
    return false;  // Core0 unresponsive
  }
  
  return cmd.valid;
}

/**
 * Send telemetry data from Core0 to Core1 (thread-safe)
 */
void sendTelemetryData(uint16_t distance, float bias, float speed, 
                       float leftSpeed, float rightSpeed, const ToFSample* sensors) {
  mutex_enter_blocking(&telemetryMutex);
  sharedTelemetry.distance = distance;
  sharedTelemetry.bias = bias;
  sharedTelemetry.speed = speed;
  sharedTelemetry.leftSpeed = leftSpeed;
  sharedTelemetry.rightSpeed = rightSpeed;
  for (uint8_t i = 0; i < 5; i++) {
    sharedTelemetry.sensors[i] = sensors[i].valid ? sensors[i].distanceMm : 0;
  }
  sharedTelemetry.timestamp = millis();
  sharedTelemetry.valid = true;
  mutex_exit(&telemetryMutex);
}

/**
 * Read telemetry data on Core1 (thread-safe)
 */
bool readTelemetryData(TelemetryData& data) {
  mutex_enter_blocking(&telemetryMutex);
  data = sharedTelemetry;
  mutex_exit(&telemetryMutex);
  return data.valid;
}

// ============================================================================
// CORE 0: TRACKING & SENSOR PROCESSING
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("Dual-Core Car Tracking System");
  Serial.println("========================================");
  Serial.println("Core0: Sensor reading + Tracking logic");
  Serial.println("Core1: Motor control + PWM (see setup1)");
  Serial.println("========================================\n");
  
  // Initialize mutex for inter-core communication
  mutex_init(&motorCmdMutex);
  mutex_init(&telemetryMutex);
  Serial.println("[CORE0] Mutexes initialized");
  
  // Initialize I2C bus
  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin();
  Serial.println("[CORE0] Wire1 initialized (GP2/GP3)");
  
  // Configure ToF sensors
  Serial.println("[CORE0] Configuring sensor array...");
  if (tof.configure(TOF_NUM, TOF_XSHUT_PINS, TOF_I2C_ADDR)) {
    Serial.println("[CORE0] ✓ Configuration complete");
  } else {
    Serial.println("[CORE0] ✗ Configuration failed!");
    return;
  }
  
  // Set timing parameters
  tof.setTiming(33000, 14, 10);  // 33ms budget, optimized for speed
  
  // Initialize sensors
  Serial.println("[CORE0] Powering up sensors...");
  uint8_t tofCount = tof.beginAll();
  Serial.printf("[CORE0] ✓ %d/%d sensors online\n\n", tofCount, TOF_NUM);
  
  // Report sensor status
  Serial.println("Sensor Status:");
  Serial.println("-------------");
  for (uint8_t i = 0; i < TOF_NUM; i++) {
    Serial.printf("  [%d] %-3s (GP%d, 0x%02X): %s\n", 
      i, TOF_NAMES[i], TOF_XSHUT_PINS[i], TOF_I2C_ADDR[i],
      tof.isOnline(i) ? "ONLINE ✓" : "OFFLINE ✗");
  }
  
  // System ready check
  trackingReady = (tofCount >= 3);
  
  if (trackingReady) {
    Serial.println("\n[CORE0] ✓ Tracking system operational");
    Serial.println("[CORE0] Waiting for Core1 motor initialization...\n");
    
    // Wait for Core1 to initialize motors
    delay(2000);
    
    Serial.println("=== DUAL-CORE TRACKING ACTIVE ===\n");
  } else {
    Serial.println("\n[CORE0] ✗ CRITICAL FAILURE - Tracking not ready");
    Serial.println("[CORE0] Check sensor connections and reset board");
  }
}

// ============================================================================
// TRACKING LOGIC
// ============================================================================

/**
 * Calculate steering direction based on CLOSEST object position
 * Returns: -1.0 (turn right) to +1.0 (turn left), 0.0 = centered
 * 
 * Logic: Find the closest sensor, then determine which direction to turn
 *        Closest on RIGHT → negative bias (turn right)
 *        Closest on LEFT → positive bias (turn left)
 *        Closest in MIDDLE → zero bias (centered)
 */
float calculateDirectionBias(const ToFSample* samples) {
  // Sensor indices: R45=0, R23=1, M0=2, L23=3, L45=4
  // Sensor weights: negative=right, zero=center, positive=left
  const float weights[TOF_NUM] = {
    -1.0f,  // R45 (index 0) - far right
    -0.5f,  // R23 (index 1) - near right
     0.0f,  // M0  (index 2) - center
    +0.5f,  // L23 (index 3) - near left
    +1.0f   // L45 (index 4) - far left
  };
  
  // Find the closest valid sensor
  uint16_t closestDist = DETECT_MAX_MM + 1;
  int8_t closestIndex = -1;
  
  for (uint8_t i = 0; i < TOF_NUM; i++) {
    if (samples[i].valid 
        && samples[i].distanceMm >= DETECT_MIN_MM 
        && samples[i].distanceMm <= DETECT_MAX_MM
        && samples[i].distanceMm < closestDist) {
      closestDist = samples[i].distanceMm;
      closestIndex = i;
    }
  }
  
  // No valid object detected
  if (closestIndex < 0) {
    return 0.0f;
  }
  
  // Return the direction bias based on which sensor detected the closest object
  return weights[closestIndex];
}

/**
 * Find closest valid detection distance
 * Returns: distance in mm, or 0 if no valid detection
 */
uint16_t getClosestDistance(const ToFSample* samples) {
  uint16_t closest = 0;
  for (uint8_t i = 0; i < TOF_NUM; i++) {
    if (samples[i].valid && samples[i].distanceMm >= DETECT_MIN_MM 
                          && samples[i].distanceMm <= DETECT_MAX_MM) {
      if (closest == 0 || samples[i].distanceMm < closest) {
        closest = samples[i].distanceMm;
      }
    }
  }
  return closest;
}

/**
 * Calculate speed based on middle sensor
 * Modified: Always return 0 (no forward movement, only rotation)
 */
float calculateSpeed(const ToFSample& middleSensor) {
  // Don't move forward - only rotate to face object
  return 0.0f;
}

/**
 * Apply tracking control based on sensor data (Core0)
 * Modified: Rotate in place to face object, show direction in serial
 */
void processTracking(const ToFSample* samples) {
  // Check middle sensor (index 2) for forward detection
  const ToFSample& middleSensor = samples[2];
  
  // Calculate steering direction from side sensors
  float directionBias = calculateDirectionBias(samples);
  
  // Check if we detect any object
  bool objectDetected = false;
  for (uint8_t i = 0; i < TOF_NUM; i++) {
    if (samples[i].valid && samples[i].distanceMm >= DETECT_MIN_MM 
                          && samples[i].distanceMm <= DETECT_MAX_MM) {
      objectDetected = true;
      break;
    }
  }
  
  if (objectDetected) {
    lastDetection = millis();
    
    // Find closest object for display
    uint16_t closestDist = DETECT_MAX_MM + 1;
    const char* closestSensor = "---";
    for (uint8_t i = 0; i < TOF_NUM; i++) {
      if (samples[i].valid 
          && samples[i].distanceMm >= DETECT_MIN_MM 
          && samples[i].distanceMm <= DETECT_MAX_MM
          && samples[i].distanceMm < closestDist) {
        closestDist = samples[i].distanceMm;
        closestSensor = TOF_NAMES[i];
      }
    }
    
    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;
    const char* direction = "CENTERED";
    
    // Rotate in place to face object
    if (directionBias > BIAS_DEADZONE) {
      // Object on LEFT - rotate left (left backward, right forward)
      leftSpeed = -ROTATION_SPEED;
      rightSpeed = -ROTATION_SPEED;
      direction = "← TURN LEFT";
    } else if (directionBias < -BIAS_DEADZONE) {
      // Object on RIGHT - rotate right (left forward, right backward)
      leftSpeed = ROTATION_SPEED;
      rightSpeed = ROTATION_SPEED;
      direction = "TURN RIGHT →";
    } else {
      // Object is centered - stop
      leftSpeed = 0.0f;
      rightSpeed = 0.0f;
      direction = "✓ CENTERED";
    }
    
    // Send motor commands to Core1
    sendMotorCommand(leftSpeed, rightSpeed, false);
    
    // Send telemetry data to Core1 for TCP streaming
    uint16_t middleDist = middleSensor.valid ? middleSensor.distanceMm : 0;
    sendTelemetryData(middleDist, directionBias, 0.0f, leftSpeed, rightSpeed, samples);
    
    // Debug output with direction and closest sensor
    Serial.printf("[CORE0] Direction: %-15s | Closest: %s=%4dmm | bias=%+.2f L=%+.1f R=%+.1f | ", 
                  direction, closestSensor, closestDist, directionBias, leftSpeed, rightSpeed);
    
  } else {
    // No object detected
    if (millis() - lastDetection > LOST_TIMEOUT) {
      // Lost for too long - emergency stop
      sendMotorCommand(0.0f, 0.0f, true);
      sendTelemetryData(0, 0.0f, 0.0f, 0.0f, 0.0f, samples);
      Serial.print("[CORE0] NO OBJECT (stopped) | ");
    } else {
      // Recently lost - stop and wait
      sendMotorCommand(0.0f, 0.0f, false);
      sendTelemetryData(0, 0.0f, 0.0f, 0.0f, 0.0f, samples);
      Serial.print("[CORE0] SEARCHING...        | ");
    }
  }
  
  // Print sensor readings
  for (uint8_t i = 0; i < TOF_NUM; i++) {
    Serial.printf("%s:", TOF_NAMES[i]);
    if (tofData[i].valid) {
      Serial.printf("%4d ", tofData[i].distanceMm);
    } else {
      Serial.print("---- ");
    }
  }
  Serial.println();
}

// ============================================================================
// CORE 0 MAIN LOOP (TRACKING)
// ============================================================================

void loop() {
  if (!trackingReady) {
    // System initialization failed - send emergency stop
    sendMotorCommand(0.0f, 0.0f, true);
    delay(1000);
    return;
  }
  
  unsigned long now = millis();
  
  // Read sensors at fixed interval (non-blocking)
  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;
    
    // Read all ToF sensors (this may take ~33ms, but Core1 keeps running)
    tof.readAll(tofData, DETECT_MIN_MM, DETECT_MAX_MM, 2);
    
    // Process tracking logic and send commands to Core1
    processTracking(tofData);
  }
  
  // Small delay to prevent tight spinning
  delay(10);
}

// ============================================================================
// CORE 1: MOTOR CONTROL, WiFi & TCP TELEMETRY
// ============================================================================

/**
 * Serialize telemetry data to JSON string
 */
void serializeTelemetryJSON(char* buffer, size_t bufferSize, const TelemetryData& data) {
  snprintf(buffer, bufferSize,
    "{\"dist\":%u,\"bias\":%.2f,\"speed\":%.1f,\"left\":%.1f,\"right\":%.1f,"
    "\"R45\":%u,\"R23\":%u,\"M0\":%u,\"L23\":%u,\"L45\":%u,\"timestamp\":%lu}\n",
    data.distance, data.bias, data.speed, data.leftSpeed, data.rightSpeed,
    data.sensors[0], data.sensors[1], data.sensors[2], data.sensors[3], data.sensors[4],
    data.timestamp
  );
}

/**
 * Handle TCP clients (accept new, remove disconnected, broadcast telemetry)
 */
void handleTCPClients() {
  // Accept new client connections
  WiFiClient newClient = tcpServer.available();
  if (newClient) {
    // Find empty slot
    bool added = false;
    for (uint8_t i = 0; i < 4; i++) {
      if (!connectedClients[i] || !connectedClients[i].connected()) {
        connectedClients[i] = newClient;
        clientCount++;
        Serial.printf("[CORE1] TCP client %d connected (total: %d)\n", i, clientCount);
        added = true;
        break;
      }
    }
    if (!added) {
      newClient.stop();  // Max clients reached
      Serial.println("[CORE1] TCP client rejected (max clients reached)");
    }
  }
  
  // Remove disconnected clients
  for (uint8_t i = 0; i < 4; i++) {
    if (connectedClients[i] && !connectedClients[i].connected()) {
      connectedClients[i].stop();
      connectedClients[i] = WiFiClient();  // Reset
      if (clientCount > 0) clientCount--;
      Serial.printf("[CORE1] TCP client %d disconnected (total: %d)\n", i, clientCount);
    }
  }
}

/**
 * Broadcast telemetry JSON to all connected TCP clients
 */
void broadcastTelemetry(const TelemetryData& data) {
  if (clientCount == 0) return;  // No clients, skip
  
  char jsonBuffer[256];
  serializeTelemetryJSON(jsonBuffer, sizeof(jsonBuffer), data);
  
  // Send to all connected clients
  for (uint8_t i = 0; i < 4; i++) {
    if (connectedClients[i] && connectedClients[i].connected()) {
      connectedClients[i].print(jsonBuffer);
    }
  }
}

/**
 * Core1 Setup - Initialize motors + WiFi AP + TCP server
 * Runs on Core1 after Core0 setup() completes
 */
void setup1() {
  // Small delay to let Core0 initialize mutexes first
  delay(500);
  
  Serial.println("[CORE1] Motor control + WiFi core starting...");
  
  // Initialize motors
  Serial.println("[CORE1] Initializing motors...");
  if (car.initializeMotors(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR, 
                           RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR, 
                           MOTOR_PWM_FREQ)) {
    Serial.println("[CORE1] ✓ Motors ready");
    Serial.printf("[CORE1]   Left:  PWM=GP%d, DIR=GP%d\n", LEFT_MOTOR_PWM, LEFT_MOTOR_DIR);
    Serial.printf("[CORE1]   Right: PWM=GP%d, DIR=GP%d\n", RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR);
    motorReady = true;
  } else {
    Serial.println("[CORE1] ✗ MOTOR INIT FAILED - Check pin assignments!");
    motorReady = false;
    return;
  }
  
  // Initialize WiFi AP mode
  Serial.println("\n[CORE1] Starting WiFi Access Point...");
  Serial.printf("[CORE1]   SSID: %s\n", WIFI_SSID);
  Serial.printf("[CORE1]   Password: %s\n", WIFI_PASSWORD);
  
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)) {
    IPAddress IP = WiFi.softAPIP();
    Serial.print("[CORE1] ✓ WiFi AP started at IP: ");
    Serial.println(IP);
    wifiReady = true;
  } else {
    Serial.println("[CORE1] ✗ WiFi AP FAILED to start!");
    wifiReady = false;
  }
  
  // Start TCP server
  if (wifiReady) {
    tcpServer.begin();
    Serial.printf("[CORE1] ✓ TCP telemetry server listening on port %d\n", TCP_PORT);
    Serial.println("[CORE1]   Clients can connect to: 192.168.4.1:8080");
    Serial.println("[CORE1]   JSON telemetry streaming enabled (10Hz)\n");
  }
  
  Serial.println("[CORE1] ✓ Core1 operational (50Hz motor + 10Hz telemetry)\n");
}

/**
 * Core1 Main Loop - Motor control + TCP telemetry streaming
 * Runs at 50Hz for motor updates, 10Hz for telemetry broadcast
 */
void loop1() {
  if (!motorReady) {
    // Motors failed to initialize - halt
    delay(1000);
    return;
  }
  
  unsigned long now = millis();
  
  // Update motors at fixed interval (50 Hz)
  if (now - lastMotorUpdate >= MOTOR_INTERVAL) {
    lastMotorUpdate = now;
    
    // Read latest motor command from Core0
    MotorCommand cmd;
    if (readMotorCommand(cmd)) {
      if (cmd.emergencyStop) {
        // Emergency stop commanded
        car.stop();
      } else {
        // Apply motor speeds
        car.setMotors(cmd.leftSpeed, cmd.rightSpeed);
      }
    } else {
      // Watchdog timeout or no valid command - safety stop
      car.stop();
      // Note: Don't spam serial, this runs at 50Hz
    }
  }
  
  // Handle TCP clients and broadcast telemetry at fixed interval (10 Hz)
  if (wifiReady && (now - lastTelemetryBroadcast >= TELEMETRY_INTERVAL)) {
    lastTelemetryBroadcast = now;
    
    // Handle new/disconnected clients
    handleTCPClients();
    
    // Read telemetry data from Core0 and broadcast to clients
    TelemetryData telemetry;
    if (readTelemetryData(telemetry)) {
      broadcastTelemetry(telemetry);
    }
  }
  
  // Small delay to prevent tight spinning
  delay(5);
}
