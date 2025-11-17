/**
 * Minimal Non-Blocking Sensor Example with 5 ToF Sensors
 * 
 * Quick test for ToFArray and Ads1115Sampler
 * Perfect for initial hardware validation
 * 
 * ToF Sensor Mapping:
 *   Index 0: Right 45° (R45) - XSHUT GP8 - I2C 0x30
 *   Index 1: Right 23° (R23) - XSHUT GP7 - I2C 0x31
 *   Index 2: Middle  0° (M0)  - XSHUT GP6 - I2C 0x32
 *   Index 3: Left  23° (L23)  - XSHUT GP5 - I2C 0x33
 *   Index 4: Left  45° (L45)  - XSHUT GP4 - I2C 0x34
 * 
 * NOTE: Extended initialization timing in ToFArray (_resetDelayMs=100ms, _postResetDelayMs=100ms)
 *       to stabilize sensor startup and prevent I2C conflicts
 *       I2C bus scan added to diagnose connection issues
 */

#include <Arduino.h>
#include <Wire.h>
#include "ToFArray.h"
#include "Car.h"

// ToF Configuration: 5 sensors with XSHUT pins
const uint8_t TOF_NUM = 5;
const uint8_t TOF_XSHUT_PINS[TOF_NUM] = {8, 7, 6, 5, 4};  // R45, R23, M0, L23, L45
const uint8_t TOF_I2C_ADDR[TOF_NUM] = {0x30, 0x31, 0x32, 0x33, 0x34};

// Motor configuration
constexpr uint8_t LEFT_MOTOR_PWM_PIN = 11;
constexpr uint8_t LEFT_MOTOR_DIR_PIN = 12;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 14;
constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 15;
constexpr uint32_t MOTOR_PWM_FREQ = 20000; // 20 kHz
constexpr float TURN_SPEED = 40.0f;
constexpr float ALIGN_SPEED = 30.0f;

// Sensor names for debugging
const char* TOF_NAMES[TOF_NUM] = {"R45", "R23", "M0", "L23", "L45"};

enum ObjectDirection : uint8_t {
  DIR_UNKNOWN = 0,
  DIR_RIGHT,
  DIR_CENTER,
  DIR_LEFT
};

const uint8_t RIGHT_SENSOR_COUNT = 2;
const uint8_t LEFT_SENSOR_COUNT = 2;
const uint8_t RIGHT_SENSOR_INDICES[RIGHT_SENSOR_COUNT] = {0, 1};
const uint8_t LEFT_SENSOR_INDICES[LEFT_SENSOR_COUNT] = {3, 4};
const uint8_t CENTER_SENSOR_INDEX = 2;
const uint8_t DIRECTION_HISTORY_SIZE = 3;

uint16_t minDistanceForSensors(const uint8_t* indices, uint8_t count);
ObjectDirection determineDirection();
ObjectDirection updateConfirmedDirection(ObjectDirection latest);
const char* directionToString(ObjectDirection dir);
const char* recommendedAction(ObjectDirection dir);
const char* applyMotorCommand(ObjectDirection dir);

// Sensor / motor objects
ToFArray tof(&Wire1,nullptr);
Car car;
const char* lastMotorCommand = "STOP";

// Data storage
ToFSample tofData[TOF_NUM];
int8_t closestSensor = -1;
uint16_t closestDistance = 0;
ObjectDirection latestDirection = DIR_UNKNOWN;
ObjectDirection confirmedDirection = DIR_UNKNOWN;
ObjectDirection directionHistory[DIRECTION_HISTORY_SIZE] = {DIR_UNKNOWN, DIR_UNKNOWN, DIR_UNKNOWN};
uint8_t directionHistoryIndex = 0;
uint8_t directionHistoryFilled = 0;

// Timing (milliseconds)
unsigned long lastRead = 0;
const uint32_t READ_INTERVAL = 100;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  Serial.println("Minimal Sensor Test");
  Serial.println("==================");
  
  // Initialize motors
  if (!car.initializeMotors(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN,
                            RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN,
                            MOTOR_PWM_FREQ)) {
    Serial.println("Motors: FAILED to initialize. Check wiring and pins.");
    while (true) {
      delay(1000);
    }
  }
  car.stop();
  Serial.println("Motors: Ready");

  // Configure and initialize ToF sensor
  if (tof.configure(TOF_NUM, TOF_XSHUT_PINS, TOF_I2C_ADDR)) {
    Serial.println("ToF: Configured");
  } else {
    Serial.println("ToF: Configuration failed!");
    return;
  }
  
  // Set timing parameters (33ms budget, pre-range=14, final-range=10)
  tof.setTiming(33000, 14, 10);
  
  // Power up and initialize sensor
  uint8_t tofCount = tof.beginAll();
  Serial.printf("ToF: %d/%d sensors online\n", tofCount, TOF_NUM);

  // Report which sensors are online
  Serial.println("\nSensor Status:");
  for (uint8_t i = 0; i < TOF_NUM; i++) {
    Serial.printf("  [%d] %s (GP%d, 0x%02X): %s\n", 
      i, TOF_NAMES[i], TOF_XSHUT_PINS[i], TOF_I2C_ADDR[i],
      tof.isOnline(i) ? "ONLINE" : "OFFLINE");
  }
  
  // I2C Bus Scanner - detect all devices
  Serial.println("\nI2C Bus Scan:");
  Serial.println("-------------");
  uint8_t nDevices = 0;
  
  for (uint8_t address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    uint8_t error = Wire1.endTransmission();
 
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.printf("Scan complete: %d device(s) found\n", nDevices);
  }
 
  delay(2000);  // Wait 2s before starting main loop
  Serial.println();
  car.forward(1);
  delay(2000);
  car.turnRight(TURN_SPEED);
  delay(300);
  car.forward(1);
  delay(500);
}

void loop() {
  unsigned long now = millis();
  
  // Read every 100ms
  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;
    
    // Read ToF sensors
    tof.readAll(tofData, 5, 1500, 2);
    closestSensor = -1;
    closestDistance = 0;
    for (uint8_t i = 0; i < TOF_NUM; i++) {
      if (!tofData[i].valid) continue;
      if (closestSensor == -1 || tofData[i].distanceMm < closestDistance) {
        closestSensor = i;
        closestDistance = tofData[i].distanceMm;
      }
    }
    latestDirection = determineDirection();
    confirmedDirection = updateConfirmedDirection(latestDirection);
    
    // Print ToF with sensor names and status codes
    Serial.print("| ToF: ");
    for (uint8_t i = 0; i < TOF_NUM; i++) {
      Serial.printf("%s[%d]:%d", TOF_NAMES[i], i, tofData[i].distanceMm);
      if (tofData[i].valid) {
        Serial.print("✓ ");
      } else {
        Serial.printf("✗(s%d) ", tofData[i].status);  // Show error status code
      }
    }
    if (closestSensor != -1) {
      Serial.printf("| Closest sensor: %s (%d mm)", TOF_NAMES[closestSensor], closestDistance);
    } else {
      Serial.print("| No valid ToF readings");
    }
    Serial.printf(" | Direction(latest=%s, confirmed=%s)", directionToString(latestDirection), directionToString(confirmedDirection));
    if (confirmedDirection != DIR_UNKNOWN) {
      Serial.printf(" | Action: %s", recommendedAction(confirmedDirection));
    }
    lastMotorCommand = applyMotorCommand(confirmedDirection);
    Serial.printf(" | Motor command: %s", lastMotorCommand);
    Serial.println();
  }
  
  delay(10);
}

uint16_t minDistanceForSensors(const uint8_t* indices, uint8_t count) {
  const uint16_t INVALID_DISTANCE = 0xFFFF;
  uint16_t minDistance = INVALID_DISTANCE;
  for (uint8_t i = 0; i < count; i++) {
    uint8_t idx = indices[i];
    if (idx >= TOF_NUM) continue;
    if (!tofData[idx].valid) continue;
    if (tofData[idx].distanceMm < minDistance) {
      minDistance = tofData[idx].distanceMm;
    }
  }
  return minDistance;
}

ObjectDirection determineDirection() {
  const uint16_t INVALID_DISTANCE = 0xFFFF;
  uint16_t rightDistance = minDistanceForSensors(RIGHT_SENSOR_INDICES, RIGHT_SENSOR_COUNT);
  uint16_t leftDistance = minDistanceForSensors(LEFT_SENSOR_INDICES, LEFT_SENSOR_COUNT);
  uint16_t centerDistance = minDistanceForSensors(&CENTER_SENSOR_INDEX, 1);

  if (rightDistance == INVALID_DISTANCE && leftDistance == INVALID_DISTANCE && centerDistance == INVALID_DISTANCE) {
    return DIR_UNKNOWN;
  }

  uint16_t bestDistance = INVALID_DISTANCE;
  ObjectDirection bestDirection = DIR_UNKNOWN;

  auto consider = [&](ObjectDirection dir, uint16_t distance) {
    if (distance == INVALID_DISTANCE) return;
    if (bestDirection == DIR_UNKNOWN || distance < bestDistance) {
      bestDistance = distance;
      bestDirection = dir;
    }
  };

  consider(DIR_RIGHT, rightDistance);
  consider(DIR_CENTER, centerDistance);
  consider(DIR_LEFT, leftDistance);

  return bestDirection;
}

ObjectDirection updateConfirmedDirection(ObjectDirection latest) {
  directionHistory[directionHistoryIndex] = latest;
  directionHistoryIndex = (directionHistoryIndex + 1) % DIRECTION_HISTORY_SIZE;
  if (directionHistoryFilled < DIRECTION_HISTORY_SIZE) {
    directionHistoryFilled++;
  }

  if (latest == DIR_UNKNOWN || directionHistoryFilled < DIRECTION_HISTORY_SIZE) {
    return DIR_UNKNOWN;
  }

  for (uint8_t i = 0; i < DIRECTION_HISTORY_SIZE; i++) {
    if (directionHistory[i] != latest) {
      return DIR_UNKNOWN;
    }
  }
  return latest;
}

const char* directionToString(ObjectDirection dir) {
  switch (dir) {
    case DIR_LEFT: return "LEFT";
    case DIR_CENTER: return "CENTER";
    case DIR_RIGHT: return "RIGHT";
    default: return "UNKNOWN";
  }
}

const char* recommendedAction(ObjectDirection dir) {
  switch (dir) {
    case DIR_LEFT: return "Rotate LEFT to face object";
    case DIR_CENTER: return "Already aligned";
    case DIR_RIGHT: return "Rotate RIGHT to face object";
    default: return "Hold position";
  }
}

const char* applyMotorCommand(ObjectDirection dir) {
  if (!car.isInitialized()) {
    car.stop();
    return "MOTORS_NOT_READY";
  }

  switch (dir) {
    case DIR_LEFT:
      car.turnLeft(TURN_SPEED);
      return "TURN_LEFT";
    case DIR_RIGHT:
      car.turnRight(TURN_SPEED);
      return "TURN_RIGHT";
    case DIR_CENTER:
      car.forward(ALIGN_SPEED);
      return "FORWARD_ALIGN";
    default:
      car.stop();
      return "STOP";
  }
}