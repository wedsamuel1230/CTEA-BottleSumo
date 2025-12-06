/*
 * TcpMotorControl - Pico W TCP Motor Control System
 * 
 * Features:
 * - WiFi AP mode with TCP server (JSONL protocol)
 * - Dual motor control via existing Motor library
 * - 4x QRE1113 edge sensors via ADS1115 I2C
 * - Software E-STOP and sensor-based safety retreat
 * 
 * Architecture: Non-blocking cooperative scheduler using millis()
 * 
 * GPIO Map:
 * - Motor Left:  PWM=GP11, DIR=GP10
 * - Motor Right: PWM=GP14, DIR=GP13
 * - I2C (Wire1): SDA=GP26, SCL=GP27
 * 
 * See specs/1-tcp-motor-control/ for full documentation
 */

#include <WiFi.h>
#include "motor_controller.h"
#include "command_parser.h"
#include "sensors_ads1115.h"
#include "safety_manager.h"

// ============================================================================
// Configuration Constants
// ============================================================================

// WiFi AP Configuration
const char* AP_SSID = "BottleSumo_AP";
const char* AP_PASSWORD = "sumobot123456";  // Min 8 chars for WPA2
const uint16_t TCP_PORT = 5000;

// Timing constants (non-blocking loop)
const uint32_t SENSOR_UPDATE_INTERVAL_MS = 20;    // 50 Hz sensor sampling
const uint32_t SAFETY_CHECK_INTERVAL_MS = 10;     // 100 Hz safety evaluation
const uint32_t CLIENT_TIMEOUT_MS = 10000;          // Inactivity timeout
const uint32_t HEARTBEAT_INTERVAL_MS = 1000;      // Status log interval

// ============================================================================
// Global Objects
// ============================================================================

WiFiServer server(TCP_PORT);
WiFiClient client;

MotorController motors;
CommandParser parser;
SensorsADS1115 sensors;
SafetyManager safety;

// ============================================================================
// State Variables
// ============================================================================

bool clientConnected = false;
unsigned long lastClientActivityMs = 0;
unsigned long lastSensorUpdateMs = 0;
unsigned long lastSafetyCheckMs = 0;
unsigned long lastHeartbeatMs = 0;
uint32_t loopCounter = 0; // Track loop execution

String inputBuffer = "";

// ============================================================================
// Setup
// ============================================================================

void setup() {
  // Initialize serial for diagnostics
  Serial.begin(115200);
  delay(500); // Brief wait for serial monitor
  
  Serial.println("\n\n========================================");
  Serial.println("TCP Motor Control - Initializing");
  Serial.println("========================================\n");
  
  // Initialize motor controller
  Serial.println("[Setup] Initializing motors...");
  motors.begin(20000); // 20 kHz PWM
  
  // Initialize sensors
  Serial.println("[Setup] Initializing ADS1115 sensors...");
  if (!sensors.begin()) {
    Serial.println("[Setup] ERROR: Sensor initialization failed!");
    Serial.println("[Setup] System will continue but safety features disabled");
  } else {
    Serial.println("[Setup] Sensors ready. Run calibration via TCP command.");
  }
  
  // Start WiFi AP
  Serial.print("[Setup] Starting WiFi AP: ");
  Serial.println(AP_SSID);
  
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  if (!apStarted) {
    Serial.println("[Setup] ERROR: Failed to start AP!");
    while (1) {
      delay(1000);
    }
  }
  
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("[Setup] AP IP: ");
  Serial.println(apIP);
  
  // Start TCP server
  server.begin();
  Serial.print("[Setup] TCP server listening on port ");
  Serial.println(TCP_PORT);
  
  Serial.println("\n[Setup] Initialization complete!");
  Serial.println("========================================\n");
  Serial.flush(); // Ensure all setup messages are sent
}

// ============================================================================
// Main Loop (Non-blocking Cooperative Scheduler)
// ============================================================================

void loop() {
  unsigned long now = millis();
  loopCounter++; // Increment every loop iteration
  
  // Task 1: Handle WiFi client connections
  handleClient(now);
  
  // Task 2: Update sensors periodically
  if (now - lastSensorUpdateMs >= SENSOR_UPDATE_INTERVAL_MS) {
    lastSensorUpdateMs = now;
    sensors.update();
  }
  
  // Task 3: Safety evaluation
  if (now - lastSafetyCheckMs >= SAFETY_CHECK_INTERVAL_MS) {
    lastSafetyCheckMs = now;
    updateSafety();
  }
  
  // Task 4: Heartbeat logging
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    logHeartbeat();
  }
  
  // Task 5: Check client timeout
  if (clientConnected && (now - lastClientActivityMs > CLIENT_TIMEOUT_MS)) {
    Serial.println("[Main] Client timeout - triggering safe-off");
    safeOff();
    disconnectClient();
  }
}

// ============================================================================
// Client Handling
// ============================================================================

void handleClient(unsigned long now) {
  // Check for new client if none connected
  if (!clientConnected) {
    WiFiClient newClient = server.accept();
    if (newClient) {
      client = newClient;
      clientConnected = true;
      lastClientActivityMs = now;
      inputBuffer = "";
      
      Serial.print("[WiFi] Client connected from ");
      Serial.println(client.remoteIP());
    }
    return;
  }
  
  // Check if client is still connected
  if (!client.connected()) {
    Serial.println("[WiFi] Client disconnected");
    safeOff();
    disconnectClient();
    return;
  }
  
  // Read available data
  while (client.available()) {
    char c = client.read();
    lastClientActivityMs = now;
    
    if (c == '\n') {
      // Process complete line
      Serial.print("[WiFi] Processing command (");
      Serial.print(inputBuffer.length());
      Serial.println(" bytes)");
      processCommand(inputBuffer);
      inputBuffer = "";
    }
    else if (c != '\r') {
      inputBuffer += c;
      
      // Prevent buffer overflow
      if (inputBuffer.length() > 512) {
        String error = parser.responseError(ERR_BAD_REQUEST, "Line too long");
        sendResponse(error);
        inputBuffer = "";
      }
    }
  }
}

void disconnectClient() {
  if (clientConnected) {
    client.stop();
    clientConnected = false;
  }
}

void sendResponse(const String& response) {
  if (clientConnected && client.connected()) {
    Serial.print("[WiFi] Sending response: ");
    Serial.println(response);
    client.println(response);
    client.flush(); // Ensure data is sent immediately
  } else {
    Serial.println("[WiFi] ERROR: Cannot send response - client not connected");
  }
}

// ============================================================================
// Command Processing
// ============================================================================

void processCommand(const String& line) {
  if (line.length() == 0) return;
  
  Serial.print("[Command] Received: ");
  Serial.println(line);
  
  // Parse command
  ParsedCommand cmd = parser.parse(line);
  
  // Handle errors
  if (cmd.error != ERR_NONE) {
    String errorMsg = getErrorMessage(cmd.error);
    String response = parser.responseError(cmd.error, errorMsg);
    sendResponse(response);
    Serial.print("[Command] Error: ");
    Serial.println(errorMsg);
    return;
  }
  
  // Execute command
  switch (cmd.action) {
    case CMD_SET:
      handleSetCommand(cmd);
      break;
    case CMD_STOP:
      handleStopCommand(cmd);
      break;
    case CMD_ESTOP:
      handleEstopCommand(cmd);
      break;
    case CMD_STATUS:
      handleStatusCommand(cmd);
      break;
    case CMD_CALIBRATE:
      handleCalibrateCommand(cmd);
      break;
    default:
      break;
  }
}

void handleSetCommand(const ParsedCommand& cmd) {
  // Validate motion is safe
  if (!safety.validateMotion(cmd.motorLeft, cmd.motorRight)) {
    String response = parser.responseError(ERR_UNSAFE_STATE, "Motion blocked by safety system");
    sendResponse(response);
    return;
  }
  
  // Apply motor commands
  motors.setBoth(cmd.motorLeft, cmd.motorRight);
  sendResponse(parser.responseOk());
  
  Serial.print("[Command] SET motors: L=");
  Serial.print(cmd.motorLeft);
  Serial.print(" R=");
  Serial.println(cmd.motorRight);
}

void handleStopCommand(const ParsedCommand& cmd) {
  motors.stopAll();
  sendResponse(parser.responseOk());
  Serial.println("[Command] STOP motors");
}

void handleEstopCommand(const ParsedCommand& cmd) {
  safety.triggerEstop();
  motors.stopAll();
  sendResponse(parser.responseOk());
  Serial.println("[Command] ESTOP activated");
}

void handleStatusCommand(const ParsedCommand& cmd) {
  uint16_t rawValues[4];
  bool flags[4];
  
  for (int i = 0; i < 4; i++) {
    rawValues[i] = sensors.getRaw(i);
    flags[i] = sensors.getFlag(i);
  }
  
  String response = parser.responseStatus(
    true,  // Auth no longer required - always accessible
    motors.getLeftValue(),
    motors.getRightValue(),
    rawValues,
    flags,
    safety.isEstop(),
    safety.isLatched()
  );
  
  sendResponse(response);
}

void handleCalibrateCommand(const ParsedCommand& cmd) {
  Serial.println("[Command] Starting calibration...");
  
  bool success = false;
  if (cmd.calibrateMode == "auto") {
    success = sensors.calibrateAuto(cmd.calibrateSamples);
  } else {
    String response = parser.responseError(ERR_BAD_REQUEST, "Unsupported calibration mode");
    sendResponse(response);
    return;
  }
  
  if (success) {
    sendResponse(parser.responseOk());
  } else {
    String response = parser.responseError(ERR_INTERNAL_ERROR, "Calibration failed");
    sendResponse(response);
  }
}

String getErrorMessage(ErrorCode error) {
  switch (error) {
    case ERR_BAD_REQUEST: return "Malformed request";
    case ERR_OUT_OF_RANGE: return "Value out of range [-255, 255]";
    case ERR_UNSAFE_STATE: return "Unsafe state detected";
    case ERR_INTERNAL_ERROR: return "Internal error";
    default: return "Unknown error";
  }
}

// ============================================================================
// Safety System
// ============================================================================

void updateSafety() {
  // Update safety manager with current sensor pattern
  uint8_t pattern = sensors.getPattern();
  safety.update(pattern);
  
  // If unsafe, apply safety override
  if (!safety.isSafe()) {
    int16_t retreatLeft, retreatRight;
    if (safety.getRetreatMotion(retreatLeft, retreatRight)) {
      // Apply retreat motion if available
      if (retreatLeft != 0 || retreatRight != 0) {
        motors.setBoth(retreatLeft, retreatRight);
      } else {
        motors.stopAll();
      }
    } else {
      // Force stop on estop
      motors.stopAll();
    }
  } else {
    // Try to clear latch if conditions met
    safety.tryClearLatch();
  }
}

void safeOff() {
  // Force motors off immediately
  motors.stopAll();
  Serial.println("[Safety] Safe-off triggered");
}

// ============================================================================
// Diagnostics
// ============================================================================

void logHeartbeat() {
  Serial.print("[Heartbeat] Loops=");
  Serial.print(loopCounter);
  Serial.print(" Motors=");
  Serial.print(motors.getLeftValue());
  Serial.print(",");
  Serial.print(motors.getRightValue());
  Serial.print(" Sensors=0x");
  Serial.print(sensors.getPattern(), HEX);
  Serial.print(" Safe=");
  Serial.println(safety.isSafe() ? "Y" : "N");
}
