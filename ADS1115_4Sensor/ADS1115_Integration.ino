/*
 * ADS1115 Integration Example
 * 
 * This example shows how the ADS1115 sensor system can be integrated
 * with the existing motor control and other systems in the repository.
 * 
 * Demonstrates integration patterns similar to how line trackers 
 * are used with motor control in the repository.
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ADS1115 setup
Adafruit_ADS1115 ads;

// Sensor configuration (matching repository patterns)
constexpr uint8_t NUM_SENS = 4U;
constexpr float VREF = 4.096F;
const uint8_t sensChannels[NUM_SENS] = {0, 1, 2, 3};
const uint8_t sensMask[NUM_SENS] = {0b0001, 0b0010, 0b0100, 0b1000};

// Motor pins (example integration with motor_test patterns)
#define LeftMotorPin1 16
#define LeftMotorPin2 17
#define RightMotorPin1 18
#define RightMotorPin2 19

// State variables
uint8_t currentSensorState = 0;
bool systemActive = false;

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }
  
  // Initialize I2C (matching repository pattern)
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  
  // Initialize ADS1115
  if (!ads.begin(0x48, &Wire1)) {
    Serial.println(F("ADS1115 failed - check connections"));
    while (1) { delay(1000); }
  }
  
  ads.setGain(GAIN_ONE);
  
  // Initialize motor pins (example)
  pinMode(LeftMotorPin1, OUTPUT);
  pinMode(LeftMotorPin2, OUTPUT);
  pinMode(RightMotorPin1, OUTPUT);
  pinMode(RightMotorPin2, OUTPUT);
  
  Serial.println(F("ADS1115 + Motor Integration Ready"));
}

void loop() {
  // Read sensors using ADS1115
  currentSensorState = checkSensors(2.0);  // 2V threshold
  
  // Make decisions based on sensor state
  handleSensorState(currentSensorState);
  
  delay(100);  // Control loop timing
}

// Main sensor reading function (matches repository pattern)
int checkSensors(float thresholdV) {
  uint8_t result = 0;
  
  for (uint8_t i = 0; i < NUM_SENS; ++i) {
    int16_t adcValue = ads.readADC_SingleEnded(sensChannels[i]);
    float volts = ads.computeVolts(adcValue);
    
    if (volts >= thresholdV) {
      result |= sensMask[i];
    }
    
    Serial.print(F("Sensor"));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(volts, 2);
    Serial.print(F("V "));
  }
  
  Serial.print(F("| State: "));
  Serial.print(result, BIN);
  Serial.print(F(" ("));
  Serial.print(result);
  Serial.println(F(")"));
  
  return result;
}

// Handle different sensor states (example integration logic)
void handleSensorState(uint8_t sensorState) {
  switch (sensorState) {
    case 0b0000:  // No sensors triggered
      Serial.println(F("Action: All clear - continue forward"));
      moveForward();
      break;
      
    case 0b0001:  // Sensor 1 only
      Serial.println(F("Action: Sensor 1 triggered - turn right"));
      turnRight();
      break;
      
    case 0b0010:  // Sensor 2 only
      Serial.println(F("Action: Sensor 2 triggered - turn left"));
      turnLeft();
      break;
      
    case 0b0100:  // Sensor 3 only
      Serial.println(F("Action: Sensor 3 triggered - reverse"));
      moveBackward();
      break;
      
    case 0b1000:  // Sensor 4 only
      Serial.println(F("Action: Sensor 4 triggered - stop"));
      stopMotors();
      break;
      
    case 0b1111:  // All sensors triggered
      Serial.println(F("Action: All sensors triggered - emergency stop"));
      emergencyStop();
      break;
      
    default:
      Serial.print(F("Action: Complex state "));
      Serial.print(sensorState, BIN);
      Serial.println(F(" - custom handling"));
      handleComplexState(sensorState);
      break;
  }
}

// Motor control functions (example implementations)
void moveForward() {
  digitalWrite(LeftMotorPin1, HIGH);
  digitalWrite(LeftMotorPin2, LOW);
  digitalWrite(RightMotorPin1, HIGH);
  digitalWrite(RightMotorPin2, LOW);
}

void moveBackward() {
  digitalWrite(LeftMotorPin1, LOW);
  digitalWrite(LeftMotorPin2, HIGH);
  digitalWrite(RightMotorPin1, LOW);
  digitalWrite(RightMotorPin2, HIGH);
}

void turnLeft() {
  digitalWrite(LeftMotorPin1, LOW);
  digitalWrite(LeftMotorPin2, HIGH);
  digitalWrite(RightMotorPin1, HIGH);
  digitalWrite(RightMotorPin2, LOW);
}

void turnRight() {
  digitalWrite(LeftMotorPin1, HIGH);
  digitalWrite(LeftMotorPin2, LOW);
  digitalWrite(RightMotorPin1, LOW);
  digitalWrite(RightMotorPin2, HIGH);
}

void stopMotors() {
  digitalWrite(LeftMotorPin1, LOW);
  digitalWrite(LeftMotorPin2, LOW);
  digitalWrite(RightMotorPin1, LOW);
  digitalWrite(RightMotorPin2, LOW);
}

void emergencyStop() {
  stopMotors();
  systemActive = false;
  Serial.println(F("EMERGENCY STOP ACTIVATED"));
}

void handleComplexState(uint8_t state) {
  // Custom logic for complex sensor combinations
  if (state & 0b0011) {  // Front sensors (1 and 2)
    Serial.println(F("Front sensors active - backing up"));
    moveBackward();
    delay(500);
    stopMotors();
  } else if (state & 0b1100) {  // Rear sensors (3 and 4)
    Serial.println(F("Rear sensors active - moving forward"));
    moveForward();
    delay(500);
    stopMotors();
  } else {
    Serial.println(F("Unknown complex state - stopping"));
    stopMotors();
  }
}

// Utility functions for integration with other systems
bool isFrontClear() {
  return !(currentSensorState & 0b0011);  // Check if front sensors are clear
}

bool isRearClear() {
  return !(currentSensorState & 0b1100);  // Check if rear sensors are clear
}

uint8_t getActiveSensorCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < NUM_SENS; i++) {
    if (currentSensorState & sensMask[i]) {
      count++;
    }
  }
  return count;
}