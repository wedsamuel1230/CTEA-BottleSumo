/**
 * CAR_final.ino
 * Merged functionality of IR Edge Detection and ToF Tracking.
 * 
 * Core 0: Sensors (IR + ToF), Logic (Edge Avoidance > Tracking), Command Sending
 * Core 1: Motor Control (Receives commands from Core 0)
 */

#include <Wire.h>
#include <math.h>
#include <pico/mutex.h>
#include <WiFi.h>
#include <stdarg.h>

#include "Car.h"
#include "ToFArray.h"
#include "Ads1115Sampler.h"

// =============================================================================
// CONFIGURATION
// =============================================================================

// --- WiFi AP Configuration ---
const char* AP_SSID = "BottleSumo_AP";
const char* AP_PASSWORD = "sumobot123456";
const uint16_t TCP_PORT = 8080;

// --- Motors ---
constexpr uint8_t LEFT_MOTOR_PWM = 11;
constexpr uint8_t LEFT_MOTOR_DIR = 12;
constexpr uint8_t RIGHT_MOTOR_PWM = 14;
constexpr uint8_t RIGHT_MOTOR_DIR = 15;
constexpr uint32_t MOTOR_PWM_FREQ = 20000;

// --- ToF Sensors (Tracking) ---
constexpr uint8_t TOF_NUM = 5;
constexpr uint8_t TOF_XSHUT_PINS[TOF_NUM] = {8, 7, 6, 5, 4};
constexpr uint8_t TOF_I2C_ADDR[TOF_NUM] = {0x30, 0x31, 0x32, 0x33, 0x34};
constexpr const char* TOF_NAMES[TOF_NUM] = {"R45", "R23", "M0", "L23", "L45"};
constexpr uint8_t I2C_TOF_SDA = 2;
constexpr uint8_t I2C_TOF_SCL = 3;

// --- IR Sensors (Edge Detection) ---
constexpr uint8_t IR_SENSOR_CHANNELS = 4;
// ADS1115 is on Wire1 (same as ToF)
constexpr uint8_t ADS1115_I2C_ADDRESS = 0x48;

// Thresholds from car_ir
float ir_threshold_front = 1.5F; 
float ir_threshold_back = 3.0F;
float threshold_min = 0.0F; // Calibration: min voltage seen
float threshold_max = 5.0F; // Calibration: max voltage seen
bool auto_threshold_enabled = false;
unsigned long calibration_start = 0;
const unsigned long CALIBRATION_DURATION_MS = 3000UL; // 3 seconds to calibrate

const float ESCAPE_SPEED = 50.0F;
const float BACK_ESCAPE_SPEED = 100.0F; // Max speed for back escape

// --- Tracking Parameters ---
constexpr uint16_t DETECT_MIN_MM = 70;
constexpr uint16_t DETECT_MAX_MM = 1000;
constexpr float BIAS_DEADZONE = 0.1f;
constexpr float TURN_GAIN = 30.0f;
constexpr float SEARCH_SPIN_SPEED = 35.0f;
constexpr float LOST_HOLD_MS = 2000.0f;
constexpr uint32_t SENSOR_INTERVAL_MS = 60;
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;
constexpr float ALIGN_SPIN_SPEED = 32.0f;
constexpr float ALIGN_FINE_SPIN_SPEED = 18.0f;

// --- Auto Start ---
const unsigned long AUTO_START_DELAY_MS = 3000UL;

// --- WiFi Globals ---
WiFiServer server(TCP_PORT);
WiFiClient client;
bool clientConnected = false;
String inputBuffer = "";

// --- Test Mode ---
enum class TestMode { AUTO, TEST_MOTOR, TEST_SENSOR, CALIBRATE_IR, CALIBRATE_TOF };
TestMode gTestMode = TestMode::AUTO;
float gTestMotorLeft = 0;
float gTestMotorRight = 0;

// =============================================================================
// SHARED DATA (Core 0 <-> Core 1)
// =============================================================================
struct MotorCommand {
    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;
    uint32_t timestamp = 0;
    bool valid = false;
    bool emergencyStop = false;
};

static MotorCommand gSharedCommand;
static mutex_t gMotorMutex;

// =============================================================================
// GLOBALS
// =============================================================================
Car gCar; // Used by Core 1 for motors
ToFArray gTof(&Wire1, nullptr);
Ads1115Sampler gAdc; 

// Tracking State
ToFSample gSamples[TOF_NUM];
unsigned long gLastSensorRead = 0;
unsigned long gLastTargetSeen = 0;
enum class TrackerMode { Searching, Holding, Aligning, Centered, EdgeAvoidance };
TrackerMode gMode = TrackerMode::Searching;
uint8_t gLastSensorHit = 0xFF;

// Edge State
bool gEdgeVerifying = false;
unsigned long gEdgeVerifyStartTime = 0;
float gIrVolts[IR_SENSOR_CHANNELS]; // Store latest IR readings for telemetry

struct TargetInfo {
    bool seen = false;
    uint16_t distance = 0;
    float bias = 0.0f;
    uint8_t sensorIndex = 0xFF;
};

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

void sendResponse(const String& response) {
    if (clientConnected && client.connected()) {
        client.println(response);
        client.flush();
    }
}

void log(String msg) {
    Serial.println(msg);
    if (clientConnected && client.connected()) {
        // Simple escape for JSON
        msg.replace("\"", "\\\"");
        msg.replace("\n", " ");
        msg.replace("\r", "");
        client.println("{\"log\":\"" + msg + "\"}");
    }
}

void logPrintf(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    log(String(buffer));
}

void handleSetMode(String modeStr) {
    modeStr.trim();
    modeStr.toUpperCase();
    if (modeStr == "AUTO") gTestMode = TestMode::AUTO;
    else if (modeStr == "TEST_MOTOR") gTestMode = TestMode::TEST_MOTOR;
    else if (modeStr == "TEST_SENSOR") gTestMode = TestMode::TEST_SENSOR;
    else if (modeStr == "CALIBRATE_IR") {
        gTestMode = TestMode::CALIBRATE_IR;
        startCalibration();
    }
    else if (modeStr == "CALIBRATE_TOF") gTestMode = TestMode::CALIBRATE_TOF;
    else {
        sendResponse("{\"error\":\"unknown_mode\"}");
        return;
    }
    sendResponse("{\"ack\":\"set_mode\",\"mode\":\"" + modeStr + "\"}");
}

void handleTestMotor(String args) {
    int spaceIdx = args.indexOf(' ');
    if (spaceIdx > 0) {
        gTestMotorLeft = args.substring(0, spaceIdx).toFloat();
        gTestMotorRight = args.substring(spaceIdx + 1).toFloat();
        sendResponse("{\"ack\":\"test_motor\",\"left\":" + String(gTestMotorLeft) + ",\"right\":" + String(gTestMotorRight) + "}");
    }
}

void handleJsonCommand(String jsonStr) {
    // Simple manual JSON parsing to avoid heavy library dependency if possible, 
    // or just look for specific keys.
    // Expected: {"cmd":"set_threshold","sensor":0,"value":2.5}
    
    if (jsonStr.indexOf("\"cmd\":\"set_threshold\"") >= 0) {
        int sensorIdx = -1;
        float value = -1.0;
        
        int sensorLoc = jsonStr.indexOf("\"sensor\":");
        if (sensorLoc > 0) {
            int end = jsonStr.indexOf(',', sensorLoc);
            if (end < 0) end = jsonStr.indexOf('}', sensorLoc);
            sensorIdx = jsonStr.substring(sensorLoc + 9, end).toInt();
        }
        
        int valueLoc = jsonStr.indexOf("\"value\":");
        if (valueLoc > 0) {
            int end = jsonStr.indexOf(',', valueLoc);
            if (end < 0) end = jsonStr.indexOf('}', valueLoc);
            value = jsonStr.substring(valueLoc + 8, end).toFloat();
        }
        
        if (sensorIdx >= 0 && value >= 0) {
            // Update threshold
            // Currently we have front/back thresholds. 
            // Map 0->Back-L(A0), 1->Front-L(A1), 2->Front-R(A2), 3->Back-R(A3)
            // But code uses ir_threshold_front/back.
            // Let's update the global variables based on index.
            // 1 & 2 are Front, 0 & 3 are Back.
            if (sensorIdx == 1 || sensorIdx == 2) ir_threshold_front = value;
            if (sensorIdx == 0 || sensorIdx == 3) ir_threshold_back = value;
            
            sendResponse("{\"ack\":\"set_threshold\",\"sensor\":" + String(sensorIdx) + ",\"value\":" + String(value) + ",\"status\":\"ok\"}");
        }
    }
}

void processTcpCommand(String line) {
    line.trim();
    if (line.startsWith("{")) {
        handleJsonCommand(line);
    } else if (line.startsWith("SET_MODE")) {
        handleSetMode(line.substring(9));
    } else if (line.startsWith("TEST_MOTOR")) {
        handleTestMotor(line.substring(11));
    }
}

void handleClient() {
    if (!clientConnected) {
        WiFiClient newClient = server.accept();
        if (newClient) {
            client = newClient;
            clientConnected = true;
            inputBuffer = "";
            log("[WiFi] Client connected");
        }
        return;
    }

    if (!client.connected()) {
        client.stop();
        clientConnected = false;
        log("[WiFi] Client disconnected");
        // Safety stop
        if (gTestMode == TestMode::TEST_MOTOR) {
            gTestMotorLeft = 0;
            gTestMotorRight = 0;
        }
        return;
    }

    while (client.available()) {
        char c = client.read();
        if (c == '\n') {
            processTcpCommand(inputBuffer);
            inputBuffer = "";
        } else if (c != '\r') {
            inputBuffer += c;
        }
    }
}

void sendTelemetry() {
    if (!clientConnected || !client.connected()) return;
    
    static unsigned long lastTelemetry = 0;
    if (millis() - lastTelemetry < 100) return; // 10Hz
    lastTelemetry = millis();

    String json = "{";
    json += "\"timestamp\":" + String(millis()) + ",";
    
    // IR Sensors
    json += "\"irsensors\":{\"voltage\":[";
    for(int i=0; i<IR_SENSOR_CHANNELS; i++) {
        json += String(gIrVolts[i], 2);
        if(i < IR_SENSOR_CHANNELS-1) json += ",";
    }
    json += "],\"edge_threshold\":[";
    // Send current thresholds for visualization
    json += String(ir_threshold_back, 2) + "," + String(ir_threshold_front, 2) + "," + 
            String(ir_threshold_front, 2) + "," + String(ir_threshold_back, 2);
    json += "]},";
    
    // ToF Sensors
    json += "\"tof\":{\"distances\":[";
    for(int i=0; i<TOF_NUM; i++) { // Send first 3 for viewer compatibility or all 5
        // Viewer expects 3? Let's send all 5, viewer might ignore extra or we map.
        // Viewer code: tof_distances=list(distances)
        // Viewer UI: loops 3 times.
        // Let's send all 5.
        json += String(gSamples[i].distanceMm);
        if(i < TOF_NUM-1) json += ",";
    }
    json += "],\"valid\":[";
    for(int i=0; i<TOF_NUM; i++) {
        json += (gSamples[i].valid ? "true" : "false");
        if(i < TOF_NUM-1) json += ",";
    }
    json += "]},";
    
    // Robot State
    json += "\"test_mode\":\"";
    switch(gTestMode) {
        case TestMode::AUTO: json += "AUTO"; break;
        case TestMode::TEST_MOTOR: json += "TEST_MOTOR"; break;
        case TestMode::TEST_SENSOR: json += "TEST_SENSOR"; break;
        case TestMode::CALIBRATE_IR: json += "CALIBRATE_IR"; break;
        case TestMode::CALIBRATE_TOF: json += "CALIBRATE_TOF"; break;
    }
    json += "\",";
    
    json += "\"motors\":{\"left\":" + String(gSharedCommand.leftSpeed) + ",\"right\":" + String(gSharedCommand.rightSpeed) + "}";
    
    json += "}";
    client.println(json);
}

// Dynamic threshold functions
void startCalibration() {
    log("\n=== STARTING CALIBRATION ===");
    log("Move car over white and black surfaces for 3 seconds...");
    calibration_start = millis();
    threshold_min = 4.10F; // Reset to max
    threshold_max = 0.0F; // Reset to min
    auto_threshold_enabled = true;
}

void updateCalibration(float voltValues[]) {
    if (!auto_threshold_enabled) return;
    
    // Track min/max values
    for (int i = 0; i < IR_SENSOR_CHANNELS; i++) {
        if (voltValues[i] < threshold_min) threshold_min = voltValues[i];
        if (voltValues[i] > threshold_max) threshold_max = voltValues[i];
    }
    
    // Check if calibration time is up
    if (millis() - calibration_start >= CALIBRATION_DURATION_MS) {
        auto_threshold_enabled = false;
        // Set threshold to midpoint (same for both front and back initially)
        ir_threshold_front = (threshold_min + threshold_max) / 2.0F;
        ir_threshold_back = (threshold_min + threshold_max) / 2.0F;
        log("\n=== CALIBRATION COMPLETE ===");
        logPrintf("Min: %.3fV, Max: %.3fV\n", threshold_min, threshold_max);
        logPrintf("Front Threshold (A1,A2): %.3fV\n", ir_threshold_front);
        logPrintf("Back Threshold (A0,A3): %.3fV\n", ir_threshold_back);
        log("============================\n");
    } else {
        // Show progress
        if ((millis() - calibration_start) % 500 < 50) {
            logPrintf("Calibrating... %.1fs remaining | Min: %.3fV Max: %.3fV\n", 
                         (CALIBRATION_DURATION_MS - (millis() - calibration_start)) / 1000.0F,
                         threshold_min, threshold_max);
        }
    }
}

void processSerialCommands() {
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.equalsIgnoreCase("cal") || cmd.equalsIgnoreCase("calibrate")) {
            startCalibration();
        }
        else if (cmd.startsWith("f+")) { // Front threshold +
            ir_threshold_front += 0.1F;
            logPrintf("Front threshold increased to: %.3fV\n", ir_threshold_front);
        }
        else if (cmd.startsWith("f-")) { // Front threshold -
            ir_threshold_front -= 0.1F;
            logPrintf("Front threshold decreased to: %.3fV\n", ir_threshold_front);
        }
        else if (cmd.startsWith("f=")) { // Front threshold set
            float newThreshold = cmd.substring(2).toFloat();
            if (newThreshold > 0 && newThreshold < threshold_max) {
                ir_threshold_front = newThreshold;
                logPrintf("Front threshold set to: %.3fV\n", ir_threshold_front);
            } else {
                log("Invalid threshold value (must be 0-4.1V)");
            }
        }
        else if (cmd.startsWith("b+")) { // Back threshold +
            ir_threshold_back += 0.1F;
            logPrintf("Back threshold increased to: %.3fV\n", ir_threshold_back);
        }
        else if (cmd.startsWith("b-")) { // Back threshold -
            ir_threshold_back -= 0.1F;
            logPrintf("Back threshold decreased to: %.3fV\n", ir_threshold_back);
        }
        else if (cmd.startsWith("b=")) { // Back threshold set
            float newThreshold = cmd.substring(2).toFloat();
            if (newThreshold > 0 && newThreshold < threshold_max) {
                ir_threshold_back = newThreshold;
                logPrintf("Back threshold set to: %.3fV\n", ir_threshold_back);
            } else {
                log("Invalid threshold value (must be 0-5V)");
            }
        }
        else if (cmd.startsWith("t+")) { // Both thresholds +
            ir_threshold_front += 0.1F;
            ir_threshold_back += 0.1F;
            logPrintf("Both thresholds increased - Front: %.3fV, Back: %.3fV\n", 
                         ir_threshold_front, ir_threshold_back);
        }
        else if (cmd.startsWith("t-")) { // Both thresholds -
            ir_threshold_front -= 0.1F;
            ir_threshold_back -= 0.1F;
            logPrintf("Both thresholds decreased - Front: %.3fV, Back: %.3fV\n", 
                         ir_threshold_front, ir_threshold_back);
        }
        else if (cmd.startsWith("t=")) { // Both thresholds set
            float newThreshold = cmd.substring(2).toFloat();
            if (newThreshold > 0 && newThreshold < threshold_max) {
                ir_threshold_front = newThreshold;
                ir_threshold_back = newThreshold;
                logPrintf("Both thresholds set to: %.3fV\n", ir_threshold_front);
            } else {
                log("Invalid threshold value (must be 0-4.1V)");
            }
        }
        else if (cmd.equalsIgnoreCase("help") || cmd.equals("?")) {
            log("\n=== DYNAMIC THRESHOLD COMMANDS ===");
            log("cal       - Start auto-calibration (3 seconds)");
            log("f+        - Increase FRONT threshold by 0.1V (A1,A2)");
            log("f-        - Decrease FRONT threshold by 0.1V (A1,A2)");
            log("f=X.XXX   - Set FRONT threshold to specific value");
            log("b+        - Increase BACK threshold by 0.1V (A0,A3)");
            log("b-        - Decrease BACK threshold by 0.1V (A0,A3)");
            log("b=X.XXX   - Set BACK threshold to specific value");
            log("t+        - Increase BOTH thresholds by 0.1V");
            log("t-        - Decrease BOTH thresholds by 0.1V");
            log("t=X.XXX   - Set BOTH thresholds to specific value");
            log("help or ? - Show this menu");
            logPrintf("\nCurrent thresholds:\n");
            logPrintf("  Front (A1,A2): %.3fV\n", ir_threshold_front);
            logPrintf("  Back (A0,A3): %.3fV\n", ir_threshold_back);
            log("==================================\n");
        }
    }
}

void sendMotorCommand(float left, float right, bool emergencyStop = false, const char* reason = nullptr) {
    mutex_enter_blocking(&gMotorMutex);
    gSharedCommand.leftSpeed = left;
    gSharedCommand.rightSpeed = right;
    gSharedCommand.emergencyStop = emergencyStop;
    gSharedCommand.timestamp = millis();
    gSharedCommand.valid = true;
    mutex_exit(&gMotorMutex);

    // Optional: Debug print
    if (reason) logPrintf("[CMD] %s L=%.1f R=%.1f\n", reason, left, right);
}

const char* modeToString(TrackerMode mode) {
    switch (mode) {
        case TrackerMode::Holding: return "Hold";
        case TrackerMode::Aligning: return "Align";
        case TrackerMode::Centered: return "Center";
        case TrackerMode::EdgeAvoidance: return "Edge";
        default: return "Search";
    }
}

void printTelemetry(const TargetInfo& target) {
    logPrintf("[MODE=%s] ", modeToString(gMode));
    if (target.seen) {
        logPrintf("Tgt:%s(%umm) ", TOF_NAMES[target.sensorIndex], target.distance);
    } else {
        log("Tgt:None ");
    }
    
    // Note: To avoid flooding logs, we might want to reduce this or only log on change/interval
    // But user asked for serial output on viewer, so we send it.
    // However, printTelemetry is called in blockingMove loop every 50ms, which is fine.
    // But in processTracking it is called every loop? No, processTracking is called every SENSOR_INTERVAL_MS (60ms).
    // So ~16Hz logging. That's acceptable for local WiFi.
}

TargetInfo findClosestTarget(const ToFSample* samples) {
    static constexpr float kWeights[TOF_NUM] = {-1.0f, -0.5f, 0.0f, +0.5f, +1.0f};
    TargetInfo info;
    uint16_t closest = 0xFFFF;
    for (uint8_t i = 0; i < TOF_NUM; ++i) {
        const ToFSample& sample = samples[i];
        if (!sample.valid) continue;
        if (sample.distanceMm < DETECT_MIN_MM || sample.distanceMm > DETECT_MAX_MM) continue;
        if (sample.distanceMm < closest) {
            closest = sample.distanceMm;
            info.seen = true;
            info.distance = sample.distanceMm;
            info.bias = kWeights[i];
            info.sensorIndex = i;
        }
    }
    if (info.seen && fabsf(info.bias) < BIAS_DEADZONE) {
        info.bias = 0.0f;
    }
    return info;
}

// Blocking move for escape maneuvers (runs on Core 0)
// Sends command repeatedly to keep Core 1 watchdog happy
void blockingMove(float left, float right, unsigned long durationMs) {
    unsigned long start = millis();
    unsigned long lastTofRead = 0;
    while (millis() - start < durationMs) {
        sendMotorCommand(left, right, false, "escape");
        
        // Update and print sensors every 50ms
        if (millis() - lastTofRead > 50) {
            lastTofRead = millis();
            
            // Update IR
            int16_t raw[IR_SENSOR_CHANNELS];
            gAdc.readAll(raw, gIrVolts, IR_SENSOR_CHANNELS);
            
            // Update ToF
            gTof.readAll(gSamples, DETECT_MIN_MM, DETECT_MAX_MM, 3);
            
            // Find target for telemetry (just for display, not control)
            TargetInfo target = findClosestTarget(gSamples);
            printTelemetry(target);
        }
        
        delay(5); 
    }
}

// =============================================================================
// EDGE DETECTION LOGIC (from car_ir)
// =============================================================================

void executeEscape(uint8_t pattern) {
    gMode = TrackerMode::EdgeAvoidance;
    logPrintf("[EDGE] Pattern 0b%x -> Escaping\n", pattern);

    // Note: car_ir uses car.backward/turnRight etc.
    // We map these to blockingMove(left, right, duration)
    // car.backward(SPEED) -> left=-SPEED, right=-SPEED
    // car.turnRight(SPEED) -> left=SPEED, right=-SPEED
    // car.turnLeft(SPEED) -> left=-SPEED, right=SPEED
    // car.forward(SPEED) -> left=SPEED, right=SPEED

    switch (pattern) {
        // Single sensor edge detection - Back away then turn
        case 0b0001: // A0 (bottom-left) → Back + Turn right
            blockingMove(-ESCAPE_SPEED, -ESCAPE_SPEED, 500);
            blockingMove(ESCAPE_SPEED, -ESCAPE_SPEED, 1000);
            break;
        case 0b0010: // A1 (top-left) → Back + Turn right
            blockingMove(-ESCAPE_SPEED, -ESCAPE_SPEED, 500);
            blockingMove(ESCAPE_SPEED, -ESCAPE_SPEED, 1000);
            break;
        case 0b0100: // A2 (top-right) → Back + Turn left
            blockingMove(-ESCAPE_SPEED, -ESCAPE_SPEED, 500);
            blockingMove(-ESCAPE_SPEED, ESCAPE_SPEED, 1000);
            break;
        case 0b1000: // A3 (bottom-right) → Back + Turn left
            blockingMove(-ESCAPE_SPEED, -ESCAPE_SPEED, 500);
            blockingMove(-ESCAPE_SPEED, ESCAPE_SPEED, 1000);
            break;
            
        // Two sensors on same side - Strong back + turn
        case 0b0011: // A0+A1 (left side) → Strong turn right
            blockingMove(-ESCAPE_SPEED, -ESCAPE_SPEED, 700);
            blockingMove(ESCAPE_SPEED, -ESCAPE_SPEED, 800);
            break;
        case 0b1100: // A2+A3 (right side) → Strong turn left
            blockingMove(-ESCAPE_SPEED, -ESCAPE_SPEED, 700);
            blockingMove(-ESCAPE_SPEED, ESCAPE_SPEED, 800);
            break;
        case 0b0110: // A1+A2 (front) → Backward + Turn
            blockingMove(-ESCAPE_SPEED, -ESCAPE_SPEED, 800);
            blockingMove(ESCAPE_SPEED, -ESCAPE_SPEED, 700);
            break;
        case 0b1001: // A0+A3 (back) → Forward + Turn
            blockingMove(ESCAPE_SPEED, ESCAPE_SPEED, 800);
            blockingMove(ESCAPE_SPEED, -ESCAPE_SPEED, 700);
            break;
            
        // 3+ sensors = CRITICAL -> STOP
        case 0b0111: case 0b1011: case 0b1101: case 0b1110: case 0b1111:
            log("[EDGE] CRITICAL! 3+ Sensors out -> STOPPING");
            sendMotorCommand(0, 0, true, "critical-stop");
            delay(500); // Ensure it stops
            break;
            
        // Diagonal patterns
        case 0b0101: // A0+A2 (diagonal)
            blockingMove(-ESCAPE_SPEED, -ESCAPE_SPEED, 800);
            blockingMove(ESCAPE_SPEED, -ESCAPE_SPEED, 700);
            break;
        case 0b1010: // A1+A3 (diagonal)
            blockingMove(-ESCAPE_SPEED, -ESCAPE_SPEED, 800);
            blockingMove(-ESCAPE_SPEED, ESCAPE_SPEED, 700);
            break;
            
        default:
            blockingMove(-ESCAPE_SPEED, -ESCAPE_SPEED, 500);
            break;
    }
    // After escape, return to search/track
    gMode = TrackerMode::Searching;
    gLastTargetSeen = millis() - LOST_HOLD_MS - 1; // Force search
}

bool checkEdge() {
    int16_t raw[IR_SENSOR_CHANNELS];
    gAdc.readAll(raw, gIrVolts, IR_SENSOR_CHANNELS);

    // Update calibration if active
    updateCalibration(gIrVolts);
    
    // Don't process sensor logic during calibration
    if (auto_threshold_enabled) {
        sendMotorCommand(0, 0, false, "calibrating");
        return true; // Treat as "handled" so loop doesn't proceed to tracking
    }

    bool edge[4];
    // A0(Back-L), A1(Front-L), A2(Front-R), A3(Back-R)
    // Logic: > Threshold = EDGE (Danger)
    edge[0] = (gIrVolts[0] > ir_threshold_back);
    edge[1] = (gIrVolts[1] > ir_threshold_front);
    edge[2] = (gIrVolts[2] > ir_threshold_front);
    edge[3] = (gIrVolts[3] > ir_threshold_back);

    // Back Edge Priority (A0 or A3)
    if (edge[0] || edge[3]) {
        log("[EDGE] BACK EDGE! Forward!");
        blockingMove(BACK_ESCAPE_SPEED, BACK_ESCAPE_SPEED, 300); 
        gEdgeVerifying = false;
        return true;
    }

    uint8_t pattern = (edge[3] << 3) | (edge[2] << 2) | (edge[1] << 1) | edge[0];

    if (pattern != 0) {
        if (!gEdgeVerifying) {
            // Start verification
            gEdgeVerifying = true;
            gEdgeVerifyStartTime = millis();
            sendMotorCommand(0, 0, false, "edge-verify"); // Stop
            return true;
        } else {
            // Verifying
            // Reduced verification time from 100ms to 10ms for faster reaction
            if (millis() - gEdgeVerifyStartTime < 10) {
                // Wait
                return true;
            } else {
                // Verified!
                executeEscape(pattern);
                gEdgeVerifying = false;
                return true;
            }
        }
    } else {
        // Safe
        if (gEdgeVerifying) {
            // False alarm
            gEdgeVerifying = false;
        }
        return false;
    }
}

// =============================================================================
// TRACKING LOGIC (from car_tracking)
// =============================================================================



void processTracking() {
    unsigned long now = millis();
    TargetInfo target = findClosestTarget(gSamples);

    if (target.seen) {
        gLastTargetSeen = now;
        gLastSensorHit = target.sensorIndex;
    }

    if (target.seen) {
        float bias = target.bias;
        float absBias = fabsf(bias);
        if (absBias <= BIAS_DEADZONE) {
            gMode = TrackerMode::Centered;
            sendMotorCommand(100.0f, -100.0f, false, "center-push"); // Forward
        } else {
            float spin = (absBias > 0.5f) ? ALIGN_SPIN_SPEED : ALIGN_FINE_SPIN_SPEED;
            if (bias > 0) { // Left bias -> Turn Left
                gMode = TrackerMode::Aligning;
                sendMotorCommand(-spin, -spin, false, "align-left"); // Spin Left
            } else { // Right bias -> Turn Right
                gMode = TrackerMode::Aligning;
                sendMotorCommand(spin, spin, false, "align-right"); // Spin Right
            }
        }
    } else if (now - gLastTargetSeen < LOST_HOLD_MS) {
        gMode = TrackerMode::Holding;
        sendMotorCommand(0.0f, 0.0f, false, "hold-wait");
    } else {
        gMode = TrackerMode::Searching;
        sendMotorCommand(SEARCH_SPIN_SPEED, SEARCH_SPIN_SPEED, false, "search-spin");
    }

    printTelemetry(target);
}

// =============================================================================
// CORE 0: SETUP & LOOP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    log("\n=== BottleSumo Final: IR Edge + ToF Tracking ===\n");

    mutex_init(&gMotorMutex);
    
    // Init I2C (Shared Bus Wire1)
    Wire1.setSDA(I2C_TOF_SDA);
    Wire1.setSCL(I2C_TOF_SCL);
    Wire1.begin();

    // Init IR (ADS1115)
    // Increased data rate to 860 SPS for faster reaction
    if (!gAdc.begin(ADS1115_I2C_ADDRESS, &Wire1, GAIN_ONE, 860)) {
        log("[CORE0] ADS1115 init failed!");
    } else {
        log("[CORE0] ADS1115 initialized.");
    }

    // Init ToF
    if (!gTof.configure(TOF_NUM, TOF_XSHUT_PINS, TOF_I2C_ADDR)) {
        log("[CORE0] ToF configure failed!");
    } else {
        gTof.setTiming(33000, 14, 10);
        uint8_t online = gTof.beginAll();
        logPrintf("[CORE0] Sensors online: %u/%u\n", online, TOF_NUM);
    }
    
    // Auto Start Delay
    logPrintf("Starting in %lu seconds...\n", AUTO_START_DELAY_MS / 1000);
    
    // Start WiFi
    log("Starting WiFi AP: ");
    log(AP_SSID);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    log("AP IP: ");
    log(WiFi.softAPIP().toString());
    server.begin();
    log("TCP Server started");

    // Print help on startup
    log("\n=== Dynamic Threshold Control ===");
    log("Type 'help' or '?' for commands");
    logPrintf("Front threshold (A1,A2): %.3fV\n", ir_threshold_front);
    logPrintf("Back threshold (A0,A3): %.3fV\n", ir_threshold_back);
    log("=================================\n");

    delay(AUTO_START_DELAY_MS);
    log("GO!");

    gLastTargetSeen = millis();
}

void loop() {
    // Process serial commands for dynamic threshold control
    processSerialCommands();
    
    // Process TCP clients
    handleClient();
    sendTelemetry();

    // Handle Test Modes
    if (gTestMode == TestMode::TEST_MOTOR) {
        sendMotorCommand(gTestMotorLeft, gTestMotorRight, false, "test-motor");
        // Update sensors for telemetry
        int16_t raw[IR_SENSOR_CHANNELS];
        gAdc.readAll(raw, gIrVolts, IR_SENSOR_CHANNELS);
        gTof.readAll(gSamples, DETECT_MIN_MM, DETECT_MAX_MM, 3);
        delay(10);
        return;
    }
    
    if (gTestMode == TestMode::TEST_SENSOR || gTestMode == TestMode::CALIBRATE_IR) {
        sendMotorCommand(0, 0, false, "test-sensor");
        int16_t raw[IR_SENSOR_CHANNELS];
        gAdc.readAll(raw, gIrVolts, IR_SENSOR_CHANNELS);
        gTof.readAll(gSamples, DETECT_MIN_MM, DETECT_MAX_MM, 3);
        
        if (gTestMode == TestMode::CALIBRATE_IR) {
             updateCalibration(gIrVolts);
        }
        delay(10);
        return;
    }

    // 1. Check Edge (High Priority)
    if (checkEdge()) {
        // If edge detected, checkEdge() handles the escape (blocking)
        // Then we return to start of loop
        return;
    }

    // 2. Tracking (If safe)
    unsigned long now = millis();
    if (now - gLastSensorRead >= SENSOR_INTERVAL_MS) {
        gLastSensorRead = now;
        gTof.readAll(gSamples, DETECT_MIN_MM, DETECT_MAX_MM, 3);
        processTracking();
    }
    
    delay(1); // Reduced loop delay
}

// =============================================================================
// CORE 1: MOTOR DRIVE
// =============================================================================

void setup1() {
    delay(500);
    if (!gCar.initializeMotors(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR,
                               RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR,
                               MOTOR_PWM_FREQ)) {
        // Error
    }
    gCar.stop();
}

bool readMotorCommand(MotorCommand& out) {
    mutex_enter_blocking(&gMotorMutex);
    out = gSharedCommand;
    mutex_exit(&gMotorMutex);

    if (!out.valid) return false;
    if (millis() - out.timestamp > WATCHDOG_TIMEOUT_MS) return false;
    return true;
}

void loop1() {
    MotorCommand cmd;
    if (readMotorCommand(cmd)) {
        if (cmd.emergencyStop) {
            gCar.stop();
        } else {
            gCar.setMotors(cmd.leftSpeed, cmd.rightSpeed);
        }
    } else {
        gCar.stop();
    }
    delay(2); // Reduced motor loop delay
}
