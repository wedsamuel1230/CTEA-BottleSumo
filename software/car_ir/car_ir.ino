//this is a car ir sensor control project

#include <Wire.h>
#include "Car.h"
#include "Ads1115Sampler.h"

constexpr uint8_t LEFT_MOTOR_PWM_PIN = 11;
constexpr uint8_t LEFT_MOTOR_DIR_PIN = 12;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 14;
constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 15;
constexpr uint32_t MOTOR_FREQ = 20000; // 20 kHz
constexpr uint8_t IR_SENSOR_CHANNELS = 4;
constexpr uint8_t IR_SENSOR_PINS[IR_SENSOR_CHANNELS] = {0, 1, 2, 3}; // A0 to A3
constexpr uint8_t ADS1115_I2C_ADDRESS = 0x48;

// Dynamic threshold variables
float ir_threshold = 2.5F; // Voltage threshold for IR detection
float threshold_min = 0.0F; // Calibration: min voltage seen
float threshold_max = 5.0F; // Calibration: max voltage seen
bool auto_threshold_enabled = false;
unsigned long calibration_start = 0;
const unsigned long CALIBRATION_DURATION_MS = 3000UL; // 3 seconds to calibrate

/*
A3 (Channel 3) - Bottom-Right IR Sensor
A2 (Channel 2) - Top-Right IR Sensor
A1 (Channel 1) - Top-Left IR Sensor
A0 (Channel 0) - Bottom-Left IR Sensor

------------------------------------------------
| IR Sensor Arrangement on the Car:            |
|                                              |
|                    Front of Car              |
|       [Top-Left (A1)]   [Top-Right (A2)]     |
|               |                 |            |
|               |                 |            |
|               |                 |            |
|       [Bottom-Left (A0)] [Bottom-Right (A3)] |
------------------------------------------------

Bit Pattern Encoding: [A3][A2][A1][A0]
Example: 0b1010 means A3=1, A2=0, A1=1, A0=0

Behavior Logic:
- Voltage < threshold → Sensor detects line (bit=1)
- Voltage > threshold → Sensor sees clear path (bit=0)

Actions:
0b0000 - All clear → Forward (70%)
0b1111/0b1110/0b1101/0b1011/0b0111 - 3+ sensors → STOP
0b0001 - A0 (bottom-left) → Turn Right (30%)
0b0010 - A1 (top-left) → Turn Right (30%)
0b0100 - A2 (top-right) → Turn Left (30%)
0b1000 - A3 (bottom-right) → Turn Left (30%)
0b0011 - A0+A1 (left side) → Turn Right (50%)
0b1100 - A2+A3 (right side) → Turn Left (50%)
0b0101 - A0+A2 (diagonal) → Slight Right (40%)
0b1010 - A1+A3 (diagonal) → Slight Left (40%)
0b1001 - A0+A3 (bottom) → Forward (70%)
0b0110 - A1+A2 (top) → Backward (70%)
Other patterns → Enter search mode
*/

Car car;
Ads1115Sampler adcSampler;

// Non-blocking searching mode state
bool searchingActive = false;
uint8_t searchStep = 0; // 0: forward, 1: left, 2: right, 3: stop
unsigned long searchStepStart = 0;
const unsigned long SEARCH_STEP_DURATION_MS = 500UL;
const unsigned long SEARCH_STOP_DURATION_MS = 200UL;

void startSearchingMode() {
    if (!searchingActive) {
        searchingActive = true;
        searchStep = 0;
        searchStepStart = millis();
    }
}

void stopSearchingMode() {
    if (searchingActive) {
        searchingActive = false;
    }
    car.stop();
}

// Dynamic threshold functions
void startCalibration() {
    Serial.println("\n=== STARTING CALIBRATION ===");
    Serial.println("Move car over white and black surfaces for 3 seconds...");
    calibration_start = millis();
    threshold_min = 5.0F; // Reset to max
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
        // Set threshold to midpoint
        ir_threshold = (threshold_min + threshold_max) / 2.0F;
        Serial.println("\n=== CALIBRATION COMPLETE ===");
        Serial.printf("Min: %.3fV, Max: %.3fV\n", threshold_min, threshold_max);
        Serial.printf("New Threshold: %.3fV\n", ir_threshold);
        Serial.println("============================\n");
    } else {
        // Show progress
        if ((millis() - calibration_start) % 500 < 50) {
            Serial.printf("Calibrating... %.1fs remaining | Min: %.3fV Max: %.3fV\n", 
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
        else if (cmd.startsWith("t+")) {
            ir_threshold += 0.1F;
            Serial.printf("Threshold increased to: %.3fV\n", ir_threshold);
        }
        else if (cmd.startsWith("t-")) {
            ir_threshold -= 0.1F;
            Serial.printf("Threshold decreased to: %.3fV\n", ir_threshold);
        }
        else if (cmd.startsWith("t=")) {
            float newThreshold = cmd.substring(2).toFloat();
            if (newThreshold > 0 && newThreshold < 5.0F) {
                ir_threshold = newThreshold;
                Serial.printf("Threshold set to: %.3fV\n", ir_threshold);
            } else {
                Serial.println("Invalid threshold value (must be 0-5V)");
            }
        }
        else if (cmd.equalsIgnoreCase("help") || cmd.equals("?")) {
            Serial.println("\n=== DYNAMIC THRESHOLD COMMANDS ===");
            Serial.println("cal       - Start auto-calibration (3 seconds)");
            Serial.println("t+        - Increase threshold by 0.1V");
            Serial.println("t-        - Decrease threshold by 0.1V");
            Serial.println("t=X.XXX   - Set threshold to specific value");
            Serial.println("help or ? - Show this menu");
            Serial.printf("\nCurrent threshold: %.3fV\n", ir_threshold);
            Serial.println("==================================\n");
        }
    }
}

// Call this frequently from loop() to run the non-blocking search sequence
void updateSearchingMode() {
    if (!searchingActive) return;
    unsigned long now = millis();
    unsigned long elapsed = now - searchStepStart;

    switch (searchStep) {
        case 0: // forward
            if (elapsed < SEARCH_STEP_DURATION_MS) {
                car.forward(50.0F);
            } else {
                searchStep = 1;
                searchStepStart = now;
            }
            break;
        case 1: // turn left
            if (elapsed < SEARCH_STEP_DURATION_MS) {
                car.turnLeft(50.0F);
            } else {
                searchStep = 2;
                searchStepStart = now;
            }
            break;
        case 2: // turn right
            if (elapsed < SEARCH_STEP_DURATION_MS) {
                car.turnRight(50.0F);
            } else {
                searchStep = 3;
                searchStepStart = now;
            }
            break;
        case 3: // short stop then loop back
            if (elapsed < SEARCH_STOP_DURATION_MS) {
                car.stop();
            } else {
                // loop search sequence
                searchStep = 0;
                searchStepStart = now;
            }
            break;
        default:
            // Reset to be safe
            searchStep = 0;
            searchStepStart = now;
            break;
    }
}

void setup(){
  Serial.begin(115200);
  while (!Serial)
  {
    delay(1);
  }
  Serial.println("Car IR Sensor Control Project");
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  if (!adcSampler.begin(0x48, &Wire1, GAIN_ONE, 128)) {
      Serial.println("Failed to initialize ADS1115");
      while (1);
  }
  Serial.println("ADS1115 initialized successfully");
  car.initializeMotors(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN,
                       RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN,
                       MOTOR_FREQ);
  if (!car.isInitialized()) {
      Serial.println("Failed to initialize motors");
      while (1);
  }
  Serial.println("Motors initialized successfully");
  car.stop();
  
  // Print help on startup
  Serial.println("\n=== Dynamic Threshold Control ===");
  Serial.println("Type 'help' or '?' for commands");
  Serial.printf("Current threshold: %.3fV\n", ir_threshold);
  Serial.println("=================================\n");
}

void searching_mode(){
    // Deprecated blocking function kept for compatibility
    // Use startSearchingMode() / updateSearchingMode() instead.
    startSearchingMode();
}

void loop(){
    // Process serial commands for dynamic threshold control
    processSerialCommands();
    
    // Read IR sensor values with non-blocking call (proper timing)
    static unsigned long lastSensorRead = 0;
    if (millis() - lastSensorRead >= 100) { // Read every 100 ms
        lastSensorRead = millis();
        
        int16_t rawValues[IR_SENSOR_CHANNELS];
        float voltValues[IR_SENSOR_CHANNELS];
        adcSampler.readAll(rawValues, voltValues, IR_SENSOR_CHANNELS);
        
        // Update calibration if active
        updateCalibration(voltValues);
        
        // Don't process sensor logic during calibration
        if (auto_threshold_enabled) {
            car.stop();
            return;
        }
        
        // Determine which sensors detect EDGE (voltage > threshold = out of area)
        bool edge_detected[IR_SENSOR_CHANNELS];
        for (int i = 0; i < IR_SENSOR_CHANNELS; i++) {
            edge_detected[i] = (voltValues[i] > ir_threshold);
        }
        
        // Create bit pattern: [A3][A2][A1][A0] 
        // 1 = edge detected (danger!), 0 = safe
        uint8_t pattern = (edge_detected[3] << 3) | (edge_detected[2] << 2) | (edge_detected[1] << 1) | edge_detected[0];
        
        // Print sensor readings - simple format
        Serial.print("Sensors: ");
        for (int i = 0; i < IR_SENSOR_CHANNELS; i++) {
            Serial.printf("A%d:%.2fV ", i, voltValues[i]);
        }
        Serial.printf("| Thr:%.2fV | ", ir_threshold);
        Serial.printf("| Thr:%.2fV | ", ir_threshold);
        
        /*
        Sensor Layout:      Bit Pattern: [A3][A2][A1][A0]
             Front
          [A1]  [A2]       A1=top-left, A2=top-right
           |      |        A0=bottom-left, A3=bottom-right
          [A0]  [A3]
          
        HIGH voltage = Edge detected (OUT of safe area) - DANGER!
        LOW voltage = On table (SAFE)
        */
        
        switch (pattern) {
            case 0b0000: // All safe - move forward
                stopSearchingMode();
                Serial.println("Forward");
                car.forward(70.0F);
                break;
                
            // Single sensor edge detection - back away from that edge
            case 0b0001: // A0 (bottom-left) detects edge → Move back-right
                stopSearchingMode();
                Serial.println("A0 edge → Back Right");
                car.backward(50.0F);
                delay(100);
                car.turnRight(50.0F);
                break;
            case 0b0010: // A1 (top-left) detects edge → Move back-right
                stopSearchingMode();
                Serial.println("A1 edge → Back Right");
                car.backward(50.0F);
                delay(100);
                car.turnRight(50.0F);
                break;
            case 0b0100: // A2 (top-right) detects edge → Move back-left
                stopSearchingMode();
                Serial.println("A2 edge → Back Left");
                car.backward(50.0F);
                delay(100);
                car.turnLeft(50.0F);
                break;
            case 0b1000: // A3 (bottom-right) detects edge → Move back-left
                stopSearchingMode();
                Serial.println("A3 edge → Back Left");
                car.backward(50.0F);
                delay(100);
                car.turnLeft(50.0F);
                break;
                
            // Two sensors on same side - strong correction
            case 0b0011: // A0+A1 (left side) → Turn hard right
                stopSearchingMode();
                Serial.println("Left edge → Hard Right");
                car.backward(50.0F);
                delay(150);
                car.turnRight(70.0F);
                break;
            case 0b1100: // A2+A3 (right side) → Turn hard left
                stopSearchingMode();
                Serial.println("Right edge → Hard Left");
                car.backward(50.0F);
                delay(150);
                car.turnLeft(70.0F);
                break;
            case 0b0110: // A1+A2 (front) → Move backward
                stopSearchingMode();
                Serial.println("Front edge → Backward");
                car.backward(70.0F);
                break;
            case 0b1001: // A0+A3 (back) → Move forward
                stopSearchingMode();
                Serial.println("Back edge → Forward");
                car.forward(70.0F);
                break;
                
            // 3+ sensors = stop immediately!
            case 0b0111: case 0b1011: case 0b1101: case 0b1110: case 0b1111:
                stopSearchingMode();
                Serial.println("3+ edges → STOP!");
                car.stop();
                break;
                
            // Diagonal patterns
            case 0b0101: // A0+A2 (diagonal)
                stopSearchingMode();
                Serial.println("Diagonal edge → Back");
                car.backward(50.0F);
                break;
            case 0b1010: // A1+A3 (diagonal)
                stopSearchingMode();
                Serial.println("Diagonal edge → Back");
                car.backward(50.0F);
                break;
                
            // Other patterns
            default:
                Serial.printf("Pattern 0b%04b\n", pattern);
                car.forward(50.0F);
                break;
        }
    }
    
    // Update search mode outside sensor reading block (runs continuously when active)
    updateSearchingMode();
}