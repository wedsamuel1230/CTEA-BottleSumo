//this is a car ir sensor control project

#include <Wire.h>
#include "Car.h"
#include "Ads1115Sampler.h"

constexpr uint8_t LEFT_MOTOR_PWM_PIN = 11;
constexpr uint8_t LEFT_MOTOR_DIR_PIN = 12;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 14;
constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 15;
constexpr uint32_t MOTOR_FREQ = 20000; // 20 kHz (safe for motor drivers)
constexpr uint8_t IR_SENSOR_CHANNELS = 4;
constexpr uint8_t IR_SENSOR_PINS[IR_SENSOR_CHANNELS] = {0, 1, 2, 3}; // A0 to A3
constexpr uint8_t ADS1115_I2C_ADDRESS = 0x48;

// Dynamic threshold variables
float ir_threshold_front = 1.5F; // Voltage threshold for A1, A2 (front sensors)
float ir_threshold_back = 3.0F;  // Voltage threshold for A0, A3 (back sensors)
float threshold_min = 0.0F; // Calibration: min voltage seen
float threshold_max = 5.0F; // Calibration: max voltage seen
bool auto_threshold_enabled = false;
unsigned long calibration_start = 0;
const unsigned long CALIBRATION_DURATION_MS = 3000UL; // 3 seconds to calibrate

// Edge detection confirmation
uint8_t last_pattern = 0b0000;
bool edge_verification_mode = false; // New: waiting to verify edge after stopping
unsigned long edge_stop_time = 0;
const unsigned long EDGE_VERIFY_DELAY_MS = 100UL; // 100ms to verify after stop

// Emergency escape mode
bool emergency_mode = false;
unsigned long emergency_start = 0;
const unsigned long EMERGENCY_DURATION_MS = 1500UL; // 1.5 seconds escape
const float ESCAPE_SPEED = 50.0F; // 50% speed is enough

// Back sensor emergency (drive forward until safe)
bool back_escape_mode = false;
const float BACK_ESCAPE_SPEED = 1.0F; // Max forward speed when rear is off the edge

// Auto-start configuration
const unsigned long AUTO_START_DELAY_MS = 3000UL; // 3 second delay before auto-start
bool auto_start_enabled = true; // Set to true for standalone operation
unsigned long startup_time = 0;

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

// Execute escape maneuver based on edge pattern
void executeEscape(uint8_t pattern) {
    switch (pattern) {
        // Single sensor edge detection - Back away then turn
        case 0b0001: // A0 (bottom-left) → Back + Turn right
            Serial.println("Back + Turn Right");
            car.backward(ESCAPE_SPEED);
            delay(500);
            car.turnRight(ESCAPE_SPEED);
            delay(1000);
            break;
        case 0b0010: // A1 (top-left) → Back + Turn right
            Serial.println("Back + Turn Right");
            car.backward(ESCAPE_SPEED);
            delay(500);
            car.turnRight(ESCAPE_SPEED);
            delay(1000);
            break;
        case 0b0100: // A2 (top-right) → Back + Turn left
            Serial.println("Back + Turn Left");
            car.backward(ESCAPE_SPEED);
            delay(500);
            car.turnLeft(ESCAPE_SPEED);
            delay(1000);
            break;
        case 0b1000: // A3 (bottom-right) → Back + Turn left
            Serial.println("Back + Turn Left");
            car.backward(ESCAPE_SPEED);
            delay(500);
            car.turnLeft(ESCAPE_SPEED);
            delay(1000);
            break;
            
        // Two sensors on same side - Strong back + turn
        case 0b0011: // A0+A1 (left side) → Strong turn right
            Serial.println("Strong Back + Right");
            car.backward(ESCAPE_SPEED);
            delay(700);
            car.turnRight(ESCAPE_SPEED);
            delay(800);
            break;
        case 0b1100: // A2+A3 (right side) → Strong turn left
            Serial.println("Strong Back + Left");
            car.backward(ESCAPE_SPEED);
            delay(700);
            car.turnLeft(ESCAPE_SPEED);
            delay(800);
            break;
        case 0b0110: // A1+A2 (front) → Backward + Turn
            Serial.println("Back + Turn Right");
            car.backward(ESCAPE_SPEED);
            delay(800);
            car.turnRight(ESCAPE_SPEED);
            delay(700);
            break;
        case 0b1001: // A0+A3 (back) → Forward + Turn
            Serial.println("Forward + Turn Right");
            car.forward(ESCAPE_SPEED);
            delay(800);
            car.turnRight(ESCAPE_SPEED);
            delay(700);
            break;
            
        // 3+ sensors = CRITICAL
        case 0b0111: case 0b1011: case 0b1101: case 0b1110: case 0b1111:
            Serial.println("CRITICAL! Back + Turn");
            car.backward(ESCAPE_SPEED);
            delay(800);
            car.turnRight(ESCAPE_SPEED);
            delay(700);
            break;
            
        // Diagonal patterns
        case 0b0101: // A0+A2 (diagonal)
            Serial.println("Back + Turn Right");
            car.backward(ESCAPE_SPEED);
            delay(800);
            car.turnRight(ESCAPE_SPEED);
            delay(700);
            break;
        case 0b1010: // A1+A3 (diagonal)
            Serial.println("Back + Turn Left");
            car.backward(ESCAPE_SPEED);
            delay(800);
            car.turnLeft(ESCAPE_SPEED);
            delay(700);
            break;
            
        default:
            Serial.println("Unknown pattern → Backward");
            car.backward(ESCAPE_SPEED);
            break;
    }
}

// Dynamic threshold functions
void startCalibration() {
    Serial.println("\n=== STARTING CALIBRATION ===");
    Serial.println("Move car over white and black surfaces for 3 seconds...");
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
        Serial.println("\n=== CALIBRATION COMPLETE ===");
        Serial.printf("Min: %.3fV, Max: %.3fV\n", threshold_min, threshold_max);
        Serial.printf("Front Threshold (A1,A2): %.3fV\n", ir_threshold_front);
        Serial.printf("Back Threshold (A0,A3): %.3fV\n", ir_threshold_back);
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
        else if (cmd.startsWith("f+")) { // Front threshold +
            ir_threshold_front += 0.1F;
            Serial.printf("Front threshold increased to: %.3fV\n", ir_threshold_front);
        }
        else if (cmd.startsWith("f-")) { // Front threshold -
            ir_threshold_front -= 0.1F;
            Serial.printf("Front threshold decreased to: %.3fV\n", ir_threshold_front);
        }
        else if (cmd.startsWith("f=")) { // Front threshold set
            float newThreshold = cmd.substring(2).toFloat();
            if (newThreshold > 0 && newThreshold < threshold_max) {
                ir_threshold_front = newThreshold;
                Serial.printf("Front threshold set to: %.3fV\n", ir_threshold_front);
            } else {
                Serial.println("Invalid threshold value (must be 0-4.1V)");
            }
        }
        else if (cmd.startsWith("b+")) { // Back threshold +
            ir_threshold_back += 0.1F;
            Serial.printf("Back threshold increased to: %.3fV\n", ir_threshold_back);
        }
        else if (cmd.startsWith("b-")) { // Back threshold -
            ir_threshold_back -= 0.1F;
            Serial.printf("Back threshold decreased to: %.3fV\n", ir_threshold_back);
        }
        else if (cmd.startsWith("b=")) { // Back threshold set
            float newThreshold = cmd.substring(2).toFloat();
            if (newThreshold > 0 && newThreshold < threshold_max) {
                ir_threshold_back = newThreshold;
                Serial.printf("Back threshold set to: %.3fV\n", ir_threshold_back);
            } else {
                Serial.println("Invalid threshold value (must be 0-5V)");
            }
        }
        else if (cmd.startsWith("t+")) { // Both thresholds +
            ir_threshold_front += 0.1F;
            ir_threshold_back += 0.1F;
            Serial.printf("Both thresholds increased - Front: %.3fV, Back: %.3fV\n", 
                         ir_threshold_front, ir_threshold_back);
        }
        else if (cmd.startsWith("t-")) { // Both thresholds -
            ir_threshold_front -= 0.1F;
            ir_threshold_back -= 0.1F;
            Serial.printf("Both thresholds decreased - Front: %.3fV, Back: %.3fV\n", 
                         ir_threshold_front, ir_threshold_back);
        }
        else if (cmd.startsWith("t=")) { // Both thresholds set
            float newThreshold = cmd.substring(2).toFloat();
            if (newThreshold > 0 && newThreshold < threshold_max) {
                ir_threshold_front = newThreshold;
                ir_threshold_back = newThreshold;
                Serial.printf("Both thresholds set to: %.3fV\n", ir_threshold_front);
            } else {
                Serial.println("Invalid threshold value (must be 0-4.1V)");
            }
        }
        else if (cmd.equalsIgnoreCase("help") || cmd.equals("?")) {
            Serial.println("\n=== DYNAMIC THRESHOLD COMMANDS ===");
            Serial.println("cal       - Start auto-calibration (3 seconds)");
            Serial.println("f+        - Increase FRONT threshold by 0.1V (A1,A2)");
            Serial.println("f-        - Decrease FRONT threshold by 0.1V (A1,A2)");
            Serial.println("f=X.XXX   - Set FRONT threshold to specific value");
            Serial.println("b+        - Increase BACK threshold by 0.1V (A0,A3)");
            Serial.println("b-        - Decrease BACK threshold by 0.1V (A0,A3)");
            Serial.println("b=X.XXX   - Set BACK threshold to specific value");
            Serial.println("t+        - Increase BOTH thresholds by 0.1V");
            Serial.println("t-        - Decrease BOTH thresholds by 0.1V");
            Serial.println("t=X.XXX   - Set BOTH thresholds to specific value");
            Serial.println("help or ? - Show this menu");
            Serial.printf("\nCurrent thresholds:\n");
            Serial.printf("  Front (A1,A2): %.3fV\n", ir_threshold_front);
            Serial.printf("  Back (A0,A3): %.3fV\n", ir_threshold_back);
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
  
  // For auto-start mode, don't wait for Serial
  if (!auto_start_enabled) {
    while (!Serial) {
      delay(1);
    }
  } else {
    delay(100); // Brief delay for serial to initialize if connected
  }
  
  Serial.println("Car IR Sensor Control Project");
  
  // Record startup time
  startup_time = millis();
  
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
  if (auto_start_enabled) {
    Serial.printf("AUTO-START MODE: Car will start in %d seconds\n", AUTO_START_DELAY_MS / 1000);
    Serial.println("Disconnect USB cable after programming!");
  }
  Serial.println("Type 'help' or '?' for commands");
  Serial.printf("Front threshold (A1,A2): %.3fV\n", ir_threshold_front);
  Serial.printf("Back threshold (A0,A3): %.3fV\n", ir_threshold_back);
  Serial.println("=================================\n");
}

void searching_mode(){
    // Deprecated blocking function kept for compatibility
    // Use startSearchingMode() / updateSearchingMode() instead.
    startSearchingMode();
}

void loop(){
    // Check if still in startup delay period (for auto-start mode)
    if (auto_start_enabled && (millis() - startup_time < AUTO_START_DELAY_MS)) {
        // Show countdown
        static unsigned long last_countdown = 0;
        if (millis() - last_countdown >= 1000) {
            last_countdown = millis();
            unsigned long remaining = (AUTO_START_DELAY_MS - (millis() - startup_time)) / 1000;
            Serial.printf("Starting in %lu seconds...\n", remaining + 1);
        }
        car.stop();
        return; // Wait for delay to complete
    }
    
    // Process serial commands for dynamic threshold control
    processSerialCommands();
    
    // Check if in emergency escape mode
    if (emergency_mode) {
        if (millis() - emergency_start < EMERGENCY_DURATION_MS) {
            // Continue emergency action - no sensor processing
            return;
        } else {
            // Emergency complete
            emergency_mode = false;
            Serial.println("Emergency escape complete!");
        }
    }
    
    // Read IR sensor values with non-blocking call (proper timing)
    static unsigned long lastSensorRead = 0;
    if (millis() - lastSensorRead >= 10) { // Read every 10ms for fast response
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
        // A0, A3 use back threshold; A1, A2 use front threshold
        edge_detected[0] = (voltValues[0] > ir_threshold_back);  // A0 - back-left
        edge_detected[1] = (voltValues[1] > ir_threshold_front); // A1 - front-left
        edge_detected[2] = (voltValues[2] > ir_threshold_front); // A2 - front-right
        edge_detected[3] = (voltValues[3] > ir_threshold_back);  // A3 - back-right
        
        // Create bit pattern: [A3][A2][A1][A0] 
        // 1 = edge detected (danger!), 0 = safe
        uint8_t pattern = (edge_detected[3] << 3) | (edge_detected[2] << 2) | (edge_detected[1] << 1) | edge_detected[0];
        
        // Print sensor readings - simple format
        Serial.print("Sensors: ");
        for (int i = 0; i < IR_SENSOR_CHANNELS; i++) {
            Serial.printf("A%d:%.2fV ", i, voltValues[i]);
        }
        Serial.printf("| Thr: F:%.2fV B:%.2fV | ", ir_threshold_front, ir_threshold_back);

        const bool back_edge_active = edge_detected[0] || edge_detected[3];
        if (back_edge_active) {
            if (!back_escape_mode) {
                back_escape_mode = true;
                edge_verification_mode = false;
                emergency_mode = false;
                searchingActive = false;
                searchStep = 0;
                Serial.printf("BACK EDGE! Pattern 0b%04b → MAX FORWARD!\n", pattern);
            }
            car.forward(BACK_ESCAPE_SPEED);
            return;
        } else if (back_escape_mode) {
            back_escape_mode = false;
            Serial.println("Back sensors safe again — resuming normal logic");
        }
        
        // NEW LOGIC: Immediate stop on ANY edge detection
        if (pattern != 0b0000) {
            // Edge detected!
            if (!edge_verification_mode && !emergency_mode) {
                // First detection - STOP IMMEDIATELY
                car.stop();
                edge_verification_mode = true;
                edge_stop_time = millis();
                last_pattern = pattern;
                Serial.printf("EDGE DETECTED 0b%04b! STOPPING...\n", pattern);
                return; // Exit to stop car
            }
        }
        
        // Check if in verification mode
        if (edge_verification_mode) {
            if (millis() - edge_stop_time < EDGE_VERIFY_DELAY_MS) {
                // Still waiting for verification delay
                Serial.printf("Verifying... (pattern: 0b%04b)\n", pattern);
                return;
            }
            
            // Verification delay complete - check if still out of safe zone
            if (pattern != 0b0000) {
                // Still detecting edge - start escape!
                Serial.printf("VERIFIED! Pattern 0b%04b → ", pattern);
                edge_verification_mode = false;
                emergency_mode = true;
                emergency_start = millis();
                
                // Execute escape based on pattern
                executeEscape(pattern);
                return;
            } else {
                // False alarm - back to safe zone
                Serial.println("False alarm - back to safe zone");
                edge_verification_mode = false;
                startSearchingMode(); // Start search mode
                return;
            }
        }
        
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
                Serial.println("Safe → Forward");
                car.forward(1.0F);
                break;
                
            default:
                // This shouldn't happen as edges trigger immediate stop above
                Serial.printf("Unexpected pattern 0b%04b\n", pattern);
                break;
        }
    }
    
    // Update search mode outside sensor reading block (runs continuously when active)
    updateSearchingMode();
}