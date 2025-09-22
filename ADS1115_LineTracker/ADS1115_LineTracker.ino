/**************************************************************************
 ADS1115 Line Tracker - Enhanced 4-Channel Line Tracking System
 
 This sketch demonstrates using the ADS1115 4-channel ADC for improved
 line tracking with higher resolution and better noise immunity compared
 to the built-in Raspberry Pi Pico ADC.
 
 Features:
 - 16-bit resolution (vs 12-bit internal ADC)
 - 4 independent channels for line sensors
 - Programmable gain for optimal sensor range
 - I2C interface reduces pin usage
 - Compatible with existing BottleSumo line tracking logic
 
 Sensor Mapping:
 - Channel 0: Left sensor
 - Channel 1: Left-Center sensor  
 - Channel 2: Right-Center sensor
 - Channel 3: Right sensor
 
 Connections:
 - VDD -> 3.3V or 5V
 - GND -> GND
 - SDA -> GP26 (Pin 31)
 - SCL -> GP27 (Pin 32)
 - A0 -> Left line sensor
 - A1 -> Left-Center line sensor  
 - A2 -> Right-Center line sensor
 - A3 -> Right line sensor
 **************************************************************************/

#include <Wire.h>

// ADS1115 Configuration
#define ADS1115_ADDRESS 0x48

// ADS1115 Register addresses
#define ADS1115_REG_CONVERSION   0x00
#define ADS1115_REG_CONFIG       0x01

// Channel multiplexer settings
#define ADS1115_MUX_SINGLE_0     0x4000  // Single-ended AIN0
#define ADS1115_MUX_SINGLE_1     0x5000  // Single-ended AIN1
#define ADS1115_MUX_SINGLE_2     0x6000  // Single-ended AIN2
#define ADS1115_MUX_SINGLE_3     0x7000  // Single-ended AIN3

// Configuration settings
#define ADS1115_OS_SINGLE        0x8000  // Start single conversion
#define ADS1115_PGA_4_096V       0x0200  // +/-4.096V range (good for 3.3V-5V sensors)
#define ADS1115_MODE_SINGLE      0x0100  // Single-shot mode
#define ADS1115_DR_128SPS        0x0080  // 128 samples per second
#define ADS1115_CQUE_NONE        0x0003  // Disable comparator

// Line tracking configuration
constexpr uint8_t NUM_SENSORS = 4;
constexpr float VREF = 4.096F;  // Reference voltage for ADS1115
constexpr float ADC_MAX = 32767.0F;  // 16-bit signed maximum

// Sensor names for debugging
const char* sensorNames[NUM_SENSORS] = {"Left", "L-Center", "R-Center", "Right"};

// Sensor bit masks for result encoding (same pattern as existing code)
const uint8_t sensorMask[NUM_SENSORS] = {0b0001, 0b0010, 0b0100, 0b1000};

// Base configuration for ADS1115
uint16_t baseConfig = ADS1115_OS_SINGLE |
                      ADS1115_PGA_4_096V |
                      ADS1115_MODE_SINGLE |
                      ADS1115_DR_128SPS |
                      ADS1115_CQUE_NONE;

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for serial monitor
  
  // Initialize I2C using the same pattern as other sketches in this repo
  Wire1.setSDA(26);
  Wire1.setSCL(27);  
  Wire1.begin();
  
  Serial.println("ADS1115 Line Tracker System");
  Serial.println("============================");
  Serial.println("I2C initialized: SDA=GP26, SCL=GP27");
  
  // Check if ADS1115 is connected
  if (checkADS1115()) {
    Serial.println("ADS1115 found and ready!");
  } else {
    Serial.println("ERROR: ADS1115 not found. Check connections.");
    Serial.println("Expected I2C address: 0x48");
    while(1); // Stop here if ADS1115 not found
  }
  
  Serial.println("\nSensor Layout:");
  Serial.println("A0: Left sensor");
  Serial.println("A1: Left-Center sensor");
  Serial.println("A2: Right-Center sensor");
  Serial.println("A3: Right sensor");
  Serial.println("\nStarting line tracking...\n");
}

void loop() {
  // Use same threshold pattern as existing line tracker code
  int result = checkLineTrackerADS1115(3.0);
  
  // Display result in same format as existing code
  Serial.print("Result(Binary): ");
  Serial.println(result, BIN);
  Serial.print("Result(Decimal): ");
  Serial.println(result);
  
  // Decode result for robot behavior (same logic as existing state_combine.ino)
  decodeLineTrackingResult(result);
  
  Serial.println("------------------------");
  delay(200); // Same delay as optimized line tracker
}

// Enhanced line tracking function using ADS1115
int checkLineTrackerADS1115(float thresholdV) {
  uint8_t result = 0;
  
  Serial.println("Channel | Voltage | Status   | Sensor");
  Serial.println("--------|---------|----------|----------");
  
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    int16_t rawValue = readADS1115Channel(i);
    float voltage = convertToVoltage(rawValue);
    
    bool outOfBounds = (voltage >= thresholdV);
    if (outOfBounds) {
      result |= sensorMask[i];
    }
    
    // Display detailed sensor information
    Serial.print("   ");
    Serial.print(i);
    Serial.print("    | ");
    Serial.print(voltage, 2);
    Serial.print("V  | ");
    Serial.print(outOfBounds ? "OUT     " : "OK      ");
    Serial.print(" | ");
    Serial.println(sensorNames[i]);
  }
  
  return result;
}

// Decode line tracking result (same logic as existing code comments)
void decodeLineTrackingResult(int result) {
  Serial.print("Robot Status: ");
  
  switch (result) {
    case 0:  // 0000 - All sensors OK
      Serial.println("Safe - Continue forward");
      break;
    case 1:  // 0001 - Left sensor out
      Serial.println("Left boundary detected - Turn right");
      break;
    case 2:  // 0010 - Left-center out
      Serial.println("Left-center boundary - Slight right turn");
      break;
    case 3:  // 0011 - Left + Left-center out
      Serial.println("Strong left boundary - Sharp right turn");
      break;
    case 4:  // 0100 - Right-center out
      Serial.println("Right-center boundary - Slight left turn");
      break;
    case 5:  // 0101 - Left + Right-center out
      Serial.println("Diagonal detection - Analyze movement");
      break;
    case 6:  // 0110 - Both center sensors out
      Serial.println("Center boundary - Sharp turn needed");
      break;
    case 7:  // 0111 - Left + both center out
      Serial.println("Major left boundary - Emergency right turn");
      break;
    case 8:  // 1000 - Right sensor out
      Serial.println("Right boundary detected - Turn left");
      break;
    case 9:  // 1001 - Left + Right out
      Serial.println("Both sides detected - Emergency stop/reverse");
      break;
    case 10: // 1010 - Left-center + Right out
      Serial.println("Cross pattern - Analyze situation");
      break;
    case 11: // 1011 - Left + Left-center + Right out
      Serial.println("Complex boundary - Emergency maneuver");
      break;
    case 12: // 1100 - Both right sensors out
      Serial.println("Strong right boundary - Sharp left turn");
      break;
    case 13: // 1101 - Left + both right out
      Serial.println("L-shaped boundary - Complex turn");
      break;
    case 14: // 1110 - Right + both center out
      Serial.println("Major right boundary - Emergency left turn");
      break;
    case 15: // 1111 - All sensors out
      Serial.println("DANGER: All boundaries detected - STOP!");
      break;
    default:
      Serial.println("Unknown state");
      break;
  }
}

// Function to check if ADS1115 is connected
bool checkADS1115() {
  Wire1.beginTransmission(ADS1115_ADDRESS);
  return (Wire1.endTransmission() == 0);
}

// Function to read a specific ADS1115 channel (0-3)
int16_t readADS1115Channel(uint8_t channel) {
  uint16_t mux;
  
  // Set the multiplexer for the specified channel
  switch (channel) {
    case 0: mux = ADS1115_MUX_SINGLE_0; break;
    case 1: mux = ADS1115_MUX_SINGLE_1; break;
    case 2: mux = ADS1115_MUX_SINGLE_2; break;
    case 3: mux = ADS1115_MUX_SINGLE_3; break;
    default: mux = ADS1115_MUX_SINGLE_0; break;
  }
  
  // Create configuration with selected channel
  uint16_t config = (baseConfig & 0x8FFF) | mux;
  
  // Write configuration to start conversion
  writeRegister(ADS1115_REG_CONFIG, config);
  
  // Wait for conversion to complete (depends on data rate)
  delay(10);
  
  // Read and return conversion result
  return (int16_t)readRegister(ADS1115_REG_CONVERSION);
}

// Function to convert raw ADC value to voltage
float convertToVoltage(int16_t rawValue) {
  // For PGA setting of +/-4.096V, each bit represents 4.096V/32767
  return rawValue * (VREF / ADC_MAX);
}

// Function to write to ADS1115 register
void writeRegister(uint8_t reg, uint16_t value) {
  Wire1.beginTransmission(ADS1115_ADDRESS);
  Wire1.write(reg);
  Wire1.write((value >> 8) & 0xFF); // High byte first
  Wire1.write(value & 0xFF);        // Low byte
  Wire1.endTransmission();
}

// Function to read from ADS1115 register
uint16_t readRegister(uint8_t reg) {
  Wire1.beginTransmission(ADS1115_ADDRESS);
  Wire1.write(reg);
  Wire1.endTransmission();
  
  Wire1.requestFrom(ADS1115_ADDRESS, 2);
  if (Wire1.available() >= 2) {
    uint8_t high = Wire1.read();
    uint8_t low = Wire1.read();
    return (high << 8) | low;
  }
  return 0;
}

// Alternative function that matches the existing optimized line tracker interface
int checkLineTrackerOptimized(float thresholdV) {
  uint8_t result = 0;
  
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    int16_t rawValue = readADS1115Channel(i);
    float voltage = convertToVoltage(rawValue);
    
    if (voltage >= thresholdV) {
      result |= sensorMask[i];
    }
    
    // Quick voltage display (similar to existing opti_linetracker_func)
    Serial.print("V");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(voltage, 2);
    Serial.print("V ");
  }
  
  Serial.print(" | Result: ");
  Serial.println(result);
  return result;
}