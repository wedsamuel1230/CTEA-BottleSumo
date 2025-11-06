/**
 * Advanced Robot Control with Non-Blocking Sensors
 * 
 * Integrates:
 * - ToFArray (VL53L0X distance sensors) for obstacle detection
 * - Ads1115Sampler (analog inputs) for battery monitoring and line sensors
 * - Car (Motor control) for autonomous navigation
 * 
 * Features:
 * - Non-blocking sensor reads
 * - Autonomous obstacle avoidance
 * - Battery monitoring
 * - Real-time decision making
 */

#include <Wire.h>
#include "ToFArray.h"
#include "Ads1115Sampler.h"
#include "Car.h"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Motor Configuration
const uint8_t LEFT_PWM_PIN  = 14;
const uint8_t LEFT_DIR_PIN  = 15;
const uint8_t RIGHT_PWM_PIN = 11;
const uint8_t RIGHT_DIR_PIN = 12;
const uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz

// ToF Sensor Configuration (3 sensors: left, front, right)
const uint8_t TOF_COUNT = 3;
const uint8_t tofXshutPins[3] = {16, 17, 18};
const uint8_t tofAddresses[3] = {0x30, 0x31, 0x32};

// ADS1115 Configuration
// Channel 0: Battery voltage divider
// Channel 1: Left line sensor
// Channel 2: Center line sensor
// Channel 3: Right line sensor
const uint8_t ADS1115_ADDR = 0x48;

// Timing Configuration
const uint32_t TOF_READ_INTERVAL = 50;      // 20 Hz
const uint32_t ADC_READ_INTERVAL = 20;      // 50 Hz
const uint32_t CONTROL_UPDATE_INTERVAL = 50; // 20 Hz control loop
const uint32_t STATUS_PRINT_INTERVAL = 1000; // 1 Hz status

// ============================================================================
// SENSOR THRESHOLDS
// ============================================================================

// Distance thresholds (mm)
const uint16_t OBSTACLE_STOP_DISTANCE = 150;   // Emergency stop
const uint16_t OBSTACLE_SLOW_DISTANCE = 300;   // Slow down
const uint16_t OBSTACLE_TURN_DISTANCE = 250;   // Start turning

// Battery thresholds (volts)
const float BATTERY_CRITICAL = 3.0;  // Stop operation
const float BATTERY_LOW = 3.5;       // Reduce speed

// Line sensor thresholds (volts)
const float LINE_THRESHOLD = 2.5;    // Detect line/edge

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Car robot;
ToFArray tofSensors(&Wire, nullptr);
Ads1115Sampler adc;

// Sensor data
ToFSample tofReadings[3];
int16_t adcRawValues[4];
float adcVoltages[4];

// Timing
unsigned long lastTofRead = 0;
unsigned long lastAdcRead = 0;
unsigned long lastControlUpdate = 0;
unsigned long lastStatusPrint = 0;

// ADC channel tracking for non-blocking reads
uint8_t currentAdcChannel = 0;

// System state
bool systemInitialized = false;
bool emergencyStop = false;

// ============================================================================
// ROBOT STATE MACHINE
// ============================================================================

enum RobotMode {
  MODE_IDLE,
  MODE_AUTONOMOUS,
  MODE_EMERGENCY_STOP,
  MODE_LOW_BATTERY
};

RobotMode currentMode = MODE_IDLE;

struct SensorData {
  // ToF distances
  uint16_t distLeft;
  uint16_t distFront;
  uint16_t distRight;
  bool obstacleLeft;
  bool obstacleFront;
  bool obstacleRight;
  
  // ADC readings
  float batteryVoltage;
  float lineSensorLeft;
  float lineSensorCenter;
  float lineSensorRight;
  bool lineDetectedLeft;
  bool lineDetectedCenter;
  bool lineDetectedRight;
  
  // Timing
  unsigned long timestamp;
};

SensorData currentSensorData;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔═══════════════════════════════════════════════════════╗");
  Serial.println("║   AUTONOMOUS ROBOT - NON-BLOCKING SENSOR CONTROL     ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝\n");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("✓ I2C initialized");
  
  // Initialize Motors
  Serial.println("\nInitializing motors...");
  if (robot.initializeMotors(LEFT_PWM_PIN, LEFT_DIR_PIN, 
                              RIGHT_PWM_PIN, RIGHT_DIR_PIN, 
                              MOTOR_PWM_FREQ)) {
    Serial.println("✓ Motors initialized");
    robot.stop();
  } else {
    Serial.println("✗ Motor initialization FAILED");
    while(1) { delay(100); }
  }
  
  // Initialize ToF Array
  Serial.println("\nInitializing ToF sensors...");
  if (tofSensors.configure(TOF_COUNT, tofXshutPins, tofAddresses)) {
    tofSensors.setTiming(20000, 12, 8);  // Fast mode
    uint8_t online = tofSensors.beginAll();
    Serial.printf("✓ ToF sensors: %d/%d online\n", online, TOF_COUNT);
    
    if (online < TOF_COUNT) {
      Serial.println("⚠ Warning: Not all ToF sensors online!");
    }
  } else {
    Serial.println("✗ ToF configuration failed");
  }
  
  // Initialize ADS1115
  Serial.println("\nInitializing ADS1115...");
  if (adc.begin(ADS1115_ADDR, &Wire, GAIN_TWOTHIRDS, RATE_ADS1115_128SPS)) {
    Serial.println("✓ ADS1115 initialized");
  } else {
    Serial.println("✗ ADS1115 initialization failed");
  }
  
  systemInitialized = true;
  
  Serial.println("\n╔═══════════════════════════════════════════════════════╗");
  Serial.println("║  System Ready - Entering AUTONOMOUS mode in 3s...    ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝\n");
  
  delay(3000);
  currentMode = MODE_AUTONOMOUS;
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long currentTime = millis();
  
  // Non-blocking sensor reads
  updateSensors(currentTime);
  
  // Control loop
  if (currentTime - lastControlUpdate >= CONTROL_UPDATE_INTERVAL) {
    lastControlUpdate = currentTime;
    
    // Update robot behavior based on sensor data
    updateRobotControl();
  }
  
  // Status printing
  if (currentTime - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
    lastStatusPrint = currentTime;
    printStatus();
  }
  
  delay(1);  // Small delay for stability
}

// ============================================================================
// SENSOR UPDATE FUNCTION
// ============================================================================

void updateSensors(unsigned long currentTime) {
  // Read ToF sensors
  if (currentTime - lastTofRead >= TOF_READ_INTERVAL) {
    lastTofRead = currentTime;
    
    tofSensors.readAll(tofReadings, 30, 2000, 2);
    
    // Process ToF data
    currentSensorData.timestamp = currentTime;
    
    // Left sensor (index 0)
    if (tofReadings[0].valid) {
      currentSensorData.distLeft = tofReadings[0].distanceMm;
      currentSensorData.obstacleLeft = (tofReadings[0].distanceMm < OBSTACLE_TURN_DISTANCE);
    } else {
      currentSensorData.distLeft = 9999;
      currentSensorData.obstacleLeft = false;
    }
    
    // Front sensor (index 1)
    if (tofReadings[1].valid) {
      currentSensorData.distFront = tofReadings[1].distanceMm;
      currentSensorData.obstacleFront = (tofReadings[1].distanceMm < OBSTACLE_TURN_DISTANCE);
    } else {
      currentSensorData.distFront = 9999;
      currentSensorData.obstacleFront = false;
    }
    
    // Right sensor (index 2)
    if (tofReadings[2].valid) {
      currentSensorData.distRight = tofReadings[2].distanceMm;
      currentSensorData.obstacleRight = (tofReadings[2].distanceMm < OBSTACLE_TURN_DISTANCE);
    } else {
      currentSensorData.distRight = 9999;
      currentSensorData.obstacleRight = false;
    }
  }
  
  // Read ADC
  if (currentTime - lastAdcRead >= ADC_READ_INTERVAL) {
    lastAdcRead = currentTime;
    
    // Non-blocking ADC read: start conversion on current channel
    adc.startConversion(currentAdcChannel);
    
    // Poll for completion
    if (adc.poll()) {
      adcRawValues[currentAdcChannel] = adc.getLastResult();
      adcVoltages[currentAdcChannel] = adc.getLastResultVolts();
      
      // Advance to next channel
      currentAdcChannel = (currentAdcChannel + 1) % 4;
    }
    
    // Process ADC data
    currentSensorData.batteryVoltage = adcVoltages[0];
    currentSensorData.lineSensorLeft = adcVoltages[1];
    currentSensorData.lineSensorCenter = adcVoltages[2];
    currentSensorData.lineSensorRight = adcVoltages[3];
    
    // Line detection
    currentSensorData.lineDetectedLeft = (adcVoltages[1] > LINE_THRESHOLD);
    currentSensorData.lineDetectedCenter = (adcVoltages[2] > LINE_THRESHOLD);
    currentSensorData.lineDetectedRight = (adcVoltages[3] > LINE_THRESHOLD);
  }
}

// ============================================================================
// ROBOT CONTROL LOGIC
// ============================================================================

void updateRobotControl() {
  // Check for critical battery
  if (currentSensorData.batteryVoltage < BATTERY_CRITICAL) {
    if (currentMode != MODE_LOW_BATTERY) {
      currentMode = MODE_LOW_BATTERY;
      Serial.println("\n[CRITICAL] Battery voltage critical! Stopping.");
    }
    robot.stop();
    return;
  }
  
  // Check for emergency stop (obstacle too close in front)
  if (currentSensorData.distFront < OBSTACLE_STOP_DISTANCE) {
    if (!emergencyStop) {
      emergencyStop = true;
      Serial.println("\n[EMERGENCY] Obstacle too close! Emergency stop.");
    }
    robot.stop();
    return;
  } else {
    emergencyStop = false;
  }
  
  // Autonomous navigation mode
  if (currentMode == MODE_AUTONOMOUS) {
    autonomousNavigation();
  }
}

void autonomousNavigation() {
  // Base speed (reduced if battery is low)
  float baseSpeed = 60.0f;
  if (currentSensorData.batteryVoltage < BATTERY_LOW) {
    baseSpeed = 40.0f;
  }
  
  // Decision tree for obstacle avoidance
  
  // Case 1: Clear path ahead
  if (!currentSensorData.obstacleFront) {
    // Go straight
    robot.forward(baseSpeed);
  }
  // Case 2: Obstacle ahead, check sides
  else {
    // Slow down
    float speed = baseSpeed * 0.5f;
    
    // Turn away from obstacles
    if (!currentSensorData.obstacleLeft && currentSensorData.obstacleRight) {
      // Turn left
      robot.drive(speed, -50);  // Speed with left turn
      Serial.println("[NAV] Obstacle ahead and right - turning left");
    }
    else if (currentSensorData.obstacleLeft && !currentSensorData.obstacleRight) {
      // Turn right
      robot.drive(speed, 50);   // Speed with right turn
      Serial.println("[NAV] Obstacle ahead and left - turning right");
    }
    else if (!currentSensorData.obstacleLeft && !currentSensorData.obstacleRight) {
      // Both sides clear, choose based on distance
      if (currentSensorData.distLeft > currentSensorData.distRight) {
        robot.drive(speed, -50);
        Serial.println("[NAV] Turning left (more space)");
      } else {
        robot.drive(speed, 50);
        Serial.println("[NAV] Turning right (more space)");
      }
    }
    else {
      // Surrounded, back up and turn
      robot.backward(speed);
      Serial.println("[NAV] Surrounded - backing up");
    }
  }
  
  // Edge detection (line sensors) - override other behaviors
  if (currentSensorData.lineDetectedCenter) {
    // Edge detected in center - back up immediately
    robot.backward(50);
    Serial.println("[EDGE] Center edge detected - backing up");
  }
  else if (currentSensorData.lineDetectedLeft) {
    // Left edge - turn right
    robot.drive(40, 60);
    Serial.println("[EDGE] Left edge detected - turning right");
  }
  else if (currentSensorData.lineDetectedRight) {
    // Right edge - turn left
    robot.drive(40, -60);
    Serial.println("[EDGE] Right edge detected - turning left");
  }
}

// ============================================================================
// STATUS DISPLAY
// ============================================================================

void printStatus() {
  Serial.println("\n┌──────────────────────────────────────────────────────────┐");
  Serial.printf("│ Mode: %-48s │\n", getModeString(currentMode));
  Serial.println("├──────────────────────────────────────────────────────────┤");
  
  // ToF sensors
  Serial.printf("│ ToF L: %4d mm  │  F: %4d mm  │  R: %4d mm       │\n",
                currentSensorData.distLeft,
                currentSensorData.distFront,
                currentSensorData.distRight);
  
  // Obstacles
  Serial.printf("│ Obstacles: L:%s │ F:%s │ R:%s                        │\n",
                currentSensorData.obstacleLeft ? "YES" : "NO ",
                currentSensorData.obstacleFront ? "YES" : "NO ",
                currentSensorData.obstacleRight ? "YES" : "NO ");
  
  Serial.println("├──────────────────────────────────────────────────────────┤");
  
  // Battery
  Serial.printf("│ Battery: %.2fV ", currentSensorData.batteryVoltage);
  if (currentSensorData.batteryVoltage < BATTERY_CRITICAL) {
    Serial.print("[CRITICAL]");
  } else if (currentSensorData.batteryVoltage < BATTERY_LOW) {
    Serial.print("[LOW]     ");
  } else {
    Serial.print("[OK]      ");
  }
  Serial.println("                        │");
  
  // Line sensors
  Serial.printf("│ Line: L:%.2fV C:%.2fV R:%.2fV                     │\n",
                currentSensorData.lineSensorLeft,
                currentSensorData.lineSensorCenter,
                currentSensorData.lineSensorRight);
  
  Serial.println("└──────────────────────────────────────────────────────────┘");
}

const char* getModeString(RobotMode mode) {
  switch(mode) {
    case MODE_IDLE: return "IDLE";
    case MODE_AUTONOMOUS: return "AUTONOMOUS";
    case MODE_EMERGENCY_STOP: return "EMERGENCY STOP";
    case MODE_LOW_BATTERY: return "LOW BATTERY";
    default: return "UNKNOWN";
  }
}
