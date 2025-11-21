/**
 * CAR_final_v2.ino
 * Dual-Core Architecture with Pure State Machine
 * 
 * Core 0: Pure State Machine
 *   - Reads sensor data from shared memory
 *   - Executes decision logic (edge avoidance > tracking)
 *   - Outputs motor commands to shared memory
 * 
 * Core 1: ALL I/O Operations (Time-Sliced)
 *   - Slot 0 (0ms):   ADS1115 IR sensor read
 *   - Slot 1 (50ms):  ToF sensor 0 read
 *   - Slot 2 (100ms): ADS1115 IR sensor read
 *   - Slot 3 (150ms): ToF sensor 1 read
 *   - Slot 4 (200ms): ADS1115 IR sensor read + Button sampling
 *   - Slot 5 (250ms): ToF sensor 2 read + Button debouncing
 *   - Slot 6 (300ms): ADS1115 IR sensor read
 *   - Slot 7 (350ms): ToF sensor 3 read + Motor PWM update
 *   - Slot 8 (400ms): WiFi/TCP handling
 *   - Slot 9 (450ms): ToF sensor 4 read
 *   Total cycle: 500ms (2Hz full sensor sweep)
 */

#include <Wire.h>
#include <WiFi.h>
#include <pico/mutex.h>

#include "Car.h"
#include "ToFArray.h"
#include "Ads1115Sampler.h"
#include "ButtonManager.h"

// =============================================================================
// CONFIGURATION
// =============================================================================

// --- Hardware Pins ---
constexpr uint8_t LEFT_MOTOR_PWM = 11;
constexpr uint8_t LEFT_MOTOR_DIR = 12;
constexpr uint8_t RIGHT_MOTOR_PWM = 14;
constexpr uint8_t RIGHT_MOTOR_DIR = 15;
constexpr uint32_t MOTOR_PWM_FREQ = 20000;

constexpr uint8_t BUTTON_TEST_PIN = 16;
constexpr uint8_t BUTTON_RUN_PIN = 17;

constexpr uint8_t TOF_NUM = 5;
constexpr uint8_t TOF_XSHUT_PINS[TOF_NUM] = {8, 7, 6, 5, 4};
constexpr uint8_t TOF_I2C_ADDR[TOF_NUM] = {0x30, 0x31, 0x32, 0x33, 0x34};
constexpr const char* TOF_NAMES[TOF_NUM] = {"R45", "R23", "M0", "L23", "L45"};
constexpr uint8_t I2C_TOF_SDA = 2;
constexpr uint8_t I2C_TOF_SCL = 3;

constexpr uint8_t IR_SENSOR_CHANNELS = 4;
constexpr uint8_t ADS1115_I2C_ADDRESS = 0x48;

// --- WiFi Configuration ---
const char* AP_SSID = "BottleSumo_AP";
const char* AP_PASSWORD = "sumobot123456";
const uint16_t TCP_PORT = 8080;

// --- Time Slicing ---
constexpr uint32_t TIME_SLICE_MS = 50;      // Each slot is 50ms
constexpr uint8_t TOTAL_SLOTS = 10;         // 10 slots = 500ms cycle

// --- Detection Parameters ---
constexpr uint16_t DETECT_MIN_MM = 70;
constexpr uint16_t DETECT_MAX_MM = 1000;
constexpr float IR_THRESHOLD_FRONT = 1.5f;
constexpr float IR_THRESHOLD_BACK = 3.0f;

// --- Behavior Parameters ---
constexpr float ESCAPE_SPEED = 50.0f;
constexpr float BACK_ESCAPE_SPEED = 100.0f;
constexpr float SEARCH_SPIN_SPEED = 35.0f;
constexpr float ALIGN_SPIN_SPEED = 32.0f;
constexpr float ALIGN_FINE_SPIN_SPEED = 18.0f;
constexpr float ATTACK_SPEED = 100.0f;
constexpr float BIAS_DEADZONE = 0.1f;
constexpr uint32_t LOST_HOLD_MS = 2000;

// =============================================================================
// SHARED MEMORY STRUCTURES (Core 0 <-> Core 1)
// =============================================================================

struct SensorData {
    // IR sensors (4 channels)
    float irVoltages[IR_SENSOR_CHANNELS];
    bool irValid;
    
    // ToF sensors (5 sensors)
    uint16_t tofDistances[TOF_NUM];
    bool tofValid[TOF_NUM];
    
    // Button state
    ButtonMode buttonMode;
    
    // Timestamp
    uint32_t timestamp;
};

struct MotorCommand {
    float leftSpeed;
    float rightSpeed;
    bool emergencyStop;
    uint32_t timestamp;
};

struct TelemetryData {
    SensorData sensors;
    MotorCommand motors;
    uint8_t stateMode;
    uint32_t timestamp;
};

// Shared memory instances
static SensorData gSharedSensorData;
static MotorCommand gSharedMotorCommand;
static TelemetryData gSharedTelemetry;

// Mutexes
static mutex_t gSensorMutex;
static mutex_t gMotorMutex;
static mutex_t gTelemetryMutex;

// =============================================================================
// CORE 0: STATE MACHINE
// =============================================================================

enum class RobotState {
    IDLE,
    CALIBRATING,
    SEARCHING,
    TRACKING,
    ATTACKING,
    EDGE_AVOIDING
};

struct StateMachine {
    RobotState currentState = RobotState::IDLE;
    RobotState previousState = RobotState::IDLE;
    
    // Edge detection state
    bool edgeDetected = false;
    uint8_t edgePattern = 0;
    unsigned long edgeDetectedTime = 0;
    
    // Tracking state
    bool targetSeen = false;
    uint8_t targetSensor = 0xFF;
    uint16_t targetDistance = 0;
    float targetBias = 0.0f;
    unsigned long lastTargetSeen = 0;
    
    // Timing
    unsigned long stateEntryTime = 0;
};

static StateMachine gStateMachine;

// Read sensor data from shared memory
void readSensorData(SensorData& data) {
    mutex_enter_blocking(&gSensorMutex);
    data = gSharedSensorData;
    mutex_exit(&gSensorMutex);
}

// Write motor command to shared memory
void writeMotorCommand(float left, float right, bool estop = false) {
    mutex_enter_blocking(&gMotorMutex);
    gSharedMotorCommand.leftSpeed = left;
    gSharedMotorCommand.rightSpeed = right;
    gSharedMotorCommand.emergencyStop = estop;
    gSharedMotorCommand.timestamp = millis();
    mutex_exit(&gMotorMutex);
}

// Edge detection logic
bool detectEdge(const SensorData& sensors, uint8_t& pattern) {
    bool edge[4];
    edge[0] = sensors.irVoltages[0] > IR_THRESHOLD_BACK;  // Back-L
    edge[1] = sensors.irVoltages[1] > IR_THRESHOLD_FRONT; // Front-L
    edge[2] = sensors.irVoltages[2] > IR_THRESHOLD_FRONT; // Front-R
    edge[3] = sensors.irVoltages[3] > IR_THRESHOLD_BACK;  // Back-R
    
    pattern = (edge[3] << 3) | (edge[2] << 2) | (edge[1] << 1) | edge[0];
    return (pattern != 0);
}

// Target detection logic
bool detectTarget(const SensorData& sensors, uint8_t& targetSensor, uint16_t& distance, float& bias) {
    static constexpr float kWeights[TOF_NUM] = {-1.0f, -0.5f, 0.0f, +0.5f, +1.0f};
    
    uint16_t closest = 0xFFFF;
    bool found = false;
    
    for (uint8_t i = 0; i < TOF_NUM; ++i) {
        if (!sensors.tofValid[i]) continue;
        if (sensors.tofDistances[i] < DETECT_MIN_MM || sensors.tofDistances[i] > DETECT_MAX_MM) continue;
        
        if (sensors.tofDistances[i] < closest) {
            closest = sensors.tofDistances[i];
            targetSensor = i;
            distance = sensors.tofDistances[i];
            bias = kWeights[i];
            found = true;
        }
    }
    
    if (found && fabsf(bias) < BIAS_DEADZONE) {
        bias = 0.0f;
    }
    
    return found;
}

// State machine transitions
void transitionState(RobotState newState) {
    if (newState != gStateMachine.currentState) {
        gStateMachine.previousState = gStateMachine.currentState;
        gStateMachine.currentState = newState;
        gStateMachine.stateEntryTime = millis();
        
        Serial.print("[STATE] ");
        Serial.print((int)gStateMachine.previousState);
        Serial.print(" -> ");
        Serial.println((int)newState);
    }
}

// Execute edge avoidance behavior
void executeEdgeAvoidance(uint8_t pattern) {
    unsigned long elapsed = millis() - gStateMachine.stateEntryTime;
    
    // Multi-stage escape based on pattern
    switch (pattern) {
        case 0b0001: // Back-L
        case 0b0010: // Front-L
            if (elapsed < 500) {
                writeMotorCommand(-ESCAPE_SPEED, -ESCAPE_SPEED); // Back
            } else if (elapsed < 1500) {
                writeMotorCommand(ESCAPE_SPEED, -ESCAPE_SPEED);  // Turn right
            } else {
                transitionState(RobotState::SEARCHING);
            }
            break;
            
        case 0b0100: // Front-R
        case 0b1000: // Back-R
            if (elapsed < 500) {
                writeMotorCommand(-ESCAPE_SPEED, -ESCAPE_SPEED); // Back
            } else if (elapsed < 1500) {
                writeMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED);  // Turn left
            } else {
                transitionState(RobotState::SEARCHING);
            }
            break;
            
        case 0b0011: // Left side
            if (elapsed < 700) {
                writeMotorCommand(-ESCAPE_SPEED, -ESCAPE_SPEED); // Back
            } else if (elapsed < 1500) {
                writeMotorCommand(ESCAPE_SPEED, -ESCAPE_SPEED);  // Turn right
            } else {
                transitionState(RobotState::SEARCHING);
            }
            break;
            
        case 0b1100: // Right side
            if (elapsed < 700) {
                writeMotorCommand(-ESCAPE_SPEED, -ESCAPE_SPEED); // Back
            } else if (elapsed < 1500) {
                writeMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED);  // Turn left
            } else {
                transitionState(RobotState::SEARCHING);
            }
            break;
            
        case 0b0110: // Front
            if (elapsed < 800) {
                writeMotorCommand(-ESCAPE_SPEED, -ESCAPE_SPEED); // Back
            } else if (elapsed < 1500) {
                writeMotorCommand(ESCAPE_SPEED, -ESCAPE_SPEED);  // Turn right
            } else {
                transitionState(RobotState::SEARCHING);
            }
            break;
            
        case 0b1001: // Back
            if (elapsed < 800) {
                writeMotorCommand(BACK_ESCAPE_SPEED, -BACK_ESCAPE_SPEED); // Forward fast
            } else {
                transitionState(RobotState::SEARCHING);
            }
            break;
            
        default: // Critical: 3+ sensors
            writeMotorCommand(0, 0, true); // Emergency stop
            if (elapsed > 500) {
                transitionState(RobotState::SEARCHING);
            }
            break;
    }
}

// Core 0 main state machine
void stateMachineUpdate() {
    SensorData sensors;
    readSensorData(sensors);
    
    // Always check for edge (highest priority)
    uint8_t edgePattern = 0;
    if (detectEdge(sensors, edgePattern)) {
        if (gStateMachine.currentState != RobotState::EDGE_AVOIDING) {
            gStateMachine.edgePattern = edgePattern;
            transitionState(RobotState::EDGE_AVOIDING);
        }
    }
    
    // Execute current state
    switch (gStateMachine.currentState) {
        case RobotState::IDLE:
            writeMotorCommand(0, 0);
            if (sensors.buttonMode == MODE_RUN) {
                transitionState(RobotState::SEARCHING);
            }
            break;
            
        case RobotState::EDGE_AVOIDING:
            executeEdgeAvoidance(gStateMachine.edgePattern);
            break;
            
        case RobotState::SEARCHING: {
            // Check for target
            uint8_t targetSensor;
            uint16_t distance;
            float bias;
            
            if (detectTarget(sensors, targetSensor, distance, bias)) {
                gStateMachine.targetSensor = targetSensor;
                gStateMachine.targetDistance = distance;
                gStateMachine.targetBias = bias;
                gStateMachine.lastTargetSeen = millis();
                transitionState(RobotState::TRACKING);
            } else {
                // Spin search
                writeMotorCommand(SEARCH_SPIN_SPEED, SEARCH_SPIN_SPEED);
            }
            break;
        }
            
        case RobotState::TRACKING: {
            uint8_t targetSensor;
            uint16_t distance;
            float bias;
            
            if (detectTarget(sensors, targetSensor, distance, bias)) {
                gStateMachine.targetSensor = targetSensor;
                gStateMachine.targetDistance = distance;
                gStateMachine.targetBias = bias;
                gStateMachine.lastTargetSeen = millis();
                
                // Decide: align or attack
                float absBias = fabsf(bias);
                if (absBias <= BIAS_DEADZONE) {
                    transitionState(RobotState::ATTACKING);
                } else {
                    // Align
                    float spin = (absBias > 0.5f) ? ALIGN_SPIN_SPEED : ALIGN_FINE_SPIN_SPEED;
                    if (bias > 0) {
                        writeMotorCommand(-spin, -spin); // Turn left
                    } else {
                        writeMotorCommand(spin, spin);   // Turn right
                    }
                }
            } else {
                // Lost target
                if (millis() - gStateMachine.lastTargetSeen > LOST_HOLD_MS) {
                    transitionState(RobotState::SEARCHING);
                } else {
                    writeMotorCommand(0, 0); // Hold
                }
            }
            break;
        }
            
        case RobotState::ATTACKING: {
            uint8_t targetSensor;
            uint16_t distance;
            float bias;
            
            if (detectTarget(sensors, targetSensor, distance, bias)) {
                gStateMachine.lastTargetSeen = millis();
                
                float absBias = fabsf(bias);
                if (absBias <= BIAS_DEADZONE) {
                    // Centered - full attack
                    writeMotorCommand(ATTACK_SPEED, -ATTACK_SPEED);
                } else {
                    // Lost center - back to tracking
                    transitionState(RobotState::TRACKING);
                }
            } else {
                // Lost target
                transitionState(RobotState::TRACKING);
            }
            break;
        }
            
        case RobotState::CALIBRATING:
            writeMotorCommand(0, 0);
            break;
    }
    
    // Update telemetry
    mutex_enter_blocking(&gTelemetryMutex);
    gSharedTelemetry.sensors = sensors;
    gSharedTelemetry.motors = gSharedMotorCommand;
    gSharedTelemetry.stateMode = (uint8_t)gStateMachine.currentState;
    gSharedTelemetry.timestamp = millis();
    mutex_exit(&gTelemetryMutex);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== BottleSumo Dual-Core v2 ===");
    Serial.println("Core 0: State Machine");
    
    mutex_init(&gSensorMutex);
    mutex_init(&gMotorMutex);
    mutex_init(&gTelemetryMutex);
    
    // Initialize shared data
    memset(&gSharedSensorData, 0, sizeof(gSharedSensorData));
    memset(&gSharedMotorCommand, 0, sizeof(gSharedMotorCommand));
    
    delay(3000); // Wait for Core 1 to initialize
    Serial.println("[CORE0] Ready");
}

void loop() {
    stateMachineUpdate();
    delay(10); // 100Hz state machine update
}

// =============================================================================
// CORE 1: I/O TIME-SLICED SCHEDULER
// =============================================================================

// Core 1 globals
Car gCar;
ToFArray gTof(&Wire1, nullptr);
Ads1115Sampler gAdc;
ButtonManager gButtonMgr(BUTTON_TEST_PIN, BUTTON_RUN_PIN);
WiFiServer gServer(TCP_PORT);
WiFiClient gClient;

uint8_t gCurrentSlot = 0;
unsigned long gLastSlotTime = 0;

// Helper: Write sensor data to shared memory
void writeSensorData(const SensorData& data) {
    mutex_enter_blocking(&gSensorMutex);
    gSharedSensorData = data;
    mutex_exit(&gSensorMutex);
}

// Helper: Read motor command from shared memory
void readMotorCommand(MotorCommand& cmd) {
    mutex_enter_blocking(&gMotorMutex);
    cmd = gSharedMotorCommand;
    mutex_exit(&gMotorMutex);
}

// Time-sliced tasks
void slot0_IR_Read() {
    int16_t raw[IR_SENSOR_CHANNELS];
    float volts[IR_SENSOR_CHANNELS];
    gAdc.readAll(raw, volts, IR_SENSOR_CHANNELS);
    
    SensorData data;
    readSensorData(data); // Get current data
    memcpy(data.irVoltages, volts, sizeof(volts));
    data.irValid = true;
    data.timestamp = millis();
    writeSensorData(data);
}

void slot1_ToF0_Read() {
    ToFSample sample;
    gTof.readOne(0, &sample, DETECT_MIN_MM, DETECT_MAX_MM, 3);
    
    SensorData data;
    readSensorData(data);
    data.tofDistances[0] = sample.distanceMm;
    data.tofValid[0] = sample.valid;
    data.timestamp = millis();
    writeSensorData(data);
}

void slot2_IR_Read() {
    slot0_IR_Read(); // Same as slot 0
}

void slot3_ToF1_Read() {
    ToFSample sample;
    gTof.readOne(1, &sample, DETECT_MIN_MM, DETECT_MAX_MM, 3);
    
    SensorData data;
    readSensorData(data);
    data.tofDistances[1] = sample.distanceMm;
    data.tofValid[1] = sample.valid;
    data.timestamp = millis();
    writeSensorData(data);
}

void slot4_IR_Button_Sample() {
    slot0_IR_Read();
    gButtonMgr.sample(5); // 5ms budget
    
    SensorData data;
    readSensorData(data);
    data.buttonMode = gButtonMgr.getMode();
    writeSensorData(data);
}

void slot5_ToF2_Button_Debounce() {
    ToFSample sample;
    gTof.readOne(2, &sample, DETECT_MIN_MM, DETECT_MAX_MM, 3);
    
    SensorData data;
    readSensorData(data);
    data.tofDistances[2] = sample.distanceMm;
    data.tofValid[2] = sample.valid;
    writeSensorData(data);
    
    gButtonMgr.debounce(20); // 20ms budget
    
    readSensorData(data);
    data.buttonMode = gButtonMgr.getMode();
    data.timestamp = millis();
    writeSensorData(data);
}

void slot6_IR_Read() {
    slot0_IR_Read();
}

void slot7_ToF3_Motor_Update() {
    ToFSample sample;
    gTof.readOne(3, &sample, DETECT_MIN_MM, DETECT_MAX_MM, 3);
    
    SensorData data;
    readSensorData(data);
    data.tofDistances[3] = sample.distanceMm;
    data.tofValid[3] = sample.valid;
    data.timestamp = millis();
    writeSensorData(data);
    
    // Motor PWM update
    MotorCommand cmd;
    readMotorCommand(cmd);
    
    if (cmd.emergencyStop) {
        gCar.stop();
    } else {
        gCar.setMotors(cmd.leftSpeed, cmd.rightSpeed);
    }
}

void slot8_WiFi_Handling() {
    // Check for new clients
    if (!gClient || !gClient.connected()) {
        WiFiClient newClient = gServer.accept();
        if (newClient) {
            gClient = newClient;
            Serial.println("[WiFi] Client connected");
        }
    }
    
    // Send telemetry
    if (gClient && gClient.connected()) {
        TelemetryData telem;
        mutex_enter_blocking(&gTelemetryMutex);
        telem = gSharedTelemetry;
        mutex_exit(&gTelemetryMutex);
        
        // Simple JSON telemetry
        String json = "{\"t\":";
        json += String(telem.timestamp);
        json += ",\"ir\":[";
        for (int i = 0; i < IR_SENSOR_CHANNELS; i++) {
            json += String(telem.sensors.irVoltages[i], 2);
            if (i < IR_SENSOR_CHANNELS - 1) json += ",";
        }
        json += "],\"tof\":[";
        for (int i = 0; i < TOF_NUM; i++) {
            json += String(telem.sensors.tofDistances[i]);
            if (i < TOF_NUM - 1) json += ",";
        }
        json += "],\"m\":[";
        json += String(telem.motors.leftSpeed);
        json += ",";
        json += String(telem.motors.rightSpeed);
        json += "],\"s\":";
        json += String(telem.stateMode);
        json += "}";
        
        gClient.println(json);
    }
}

void slot9_ToF4_Read() {
    ToFSample sample;
    gTof.readOne(4, &sample, DETECT_MIN_MM, DETECT_MAX_MM, 3);
    
    SensorData data;
    readSensorData(data);
    data.tofDistances[4] = sample.distanceMm;
    data.tofValid[4] = sample.valid;
    data.timestamp = millis();
    writeSensorData(data);
}

void setup1() {
    Serial.println("\n[CORE1] Initializing I/O Scheduler");
    
    // Init I2C
    Wire1.setSDA(I2C_TOF_SDA);
    Wire1.setSCL(I2C_TOF_SCL);
    Wire1.begin();
    
    // Init ADS1115
    if (!gAdc.begin(ADS1115_I2C_ADDRESS, &Wire1, GAIN_ONE, 860)) {
        Serial.println("[CORE1] ADS1115 FAILED");
    } else {
        Serial.println("[CORE1] ADS1115 OK");
    }
    
    // Init ToF
    if (!gTof.configure(TOF_NUM, TOF_XSHUT_PINS, TOF_I2C_ADDR)) {
        Serial.println("[CORE1] ToF configure FAILED");
    } else {
        gTof.setTiming(33000, 14, 10);
        uint8_t online = gTof.beginAll();
        Serial.print("[CORE1] ToF sensors online: ");
        Serial.print(online);
        Serial.print("/");
        Serial.println(TOF_NUM);
    }
    
    // Init motors
    if (!gCar.initializeMotors(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR,
                                RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR,
                                MOTOR_PWM_FREQ)) {
        Serial.println("[CORE1] Motors FAILED");
    } else {
        Serial.println("[CORE1] Motors OK");
        gCar.stop();
    }
    
    // Init buttons
    gButtonMgr.begin();
    Serial.println("[CORE1] Buttons OK");
    
    // Init WiFi
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("[CORE1] WiFi AP: ");
    Serial.println(WiFi.softAPIP());
    gServer.begin();
    Serial.println("[CORE1] TCP Server started");
    
    gLastSlotTime = millis();
    Serial.println("[CORE1] Ready - Time-Sliced Scheduler Active");
}

void loop1() {
    unsigned long now = millis();
    
    // Check if it's time for next slot
    if (now - gLastSlotTime >= TIME_SLICE_MS) {
        gLastSlotTime = now;
        
        // Execute current slot
        switch (gCurrentSlot) {
            case 0: slot0_IR_Read(); break;
            case 1: slot1_ToF0_Read(); break;
            case 2: slot2_IR_Read(); break;
            case 3: slot3_ToF1_Read(); break;
            case 4: slot4_IR_Button_Sample(); break;
            case 5: slot5_ToF2_Button_Debounce(); break;
            case 6: slot6_IR_Read(); break;
            case 7: slot7_ToF3_Motor_Update(); break;
            case 8: slot8_WiFi_Handling(); break;
            case 9: slot9_ToF4_Read(); break;
        }
        
        // Advance to next slot
        gCurrentSlot = (gCurrentSlot + 1) % TOTAL_SLOTS;
    }
    
    delay(1); // Minimal delay
}
