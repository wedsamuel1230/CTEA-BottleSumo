/*
 * ADS1115 4-Channel ADC Sensor Reading for Raspberry Pi Pico
 * 
 * This code reads 4 ADC-based sensors connected to an ADS1115 module via I2C
 * and converts the voltage readings to meaningful values.
 * 
 * Hardware Setup:
 * - Raspberry Pi Pico
 * - ADS1115 ADC module connected via I2C
 * - I2C Pins: GP26 (SDA), GP27 (SCL)
 * - 4 analog sensors connected to ADS1115 channels A0, A1, A2, A3
 * 
 * ADS1115 Default I2C Address: 0x48
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ADS1115 configuration
Adafruit_ADS1115 ads;  // Create ADS1115 instance

// Sensor configuration
constexpr uint8_t NUM_SENSORS = 4U;
constexpr float ADS_VOLTAGE_RANGE = 4.096F;  // ADS1115 voltage range in V (±4.096V for gain 1)
constexpr int16_t ADS_MAX_VALUE = 32767;     // ADS1115 15-bit signed max value
const uint8_t sensorChannels[NUM_SENSORS] = {0, 1, 2, 3};  // ADS1115 channels A0, A1, A2, A3
const char* sensorNames[NUM_SENSORS] = {"Sensor1", "Sensor2", "Sensor3", "Sensor4"};

// Sensor values
float sensorVoltages[NUM_SENSORS];
int16_t sensorRawValues[NUM_SENSORS];

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }  // Wait for serial connection
  
  Serial.println("ADS1115 4-Channel Sensor Reader");
  Serial.println("================================");
  
  // Initialize I2C with Raspberry Pi Pico pins
  Wire1.setSDA(26);  // GP26 as SDA
  Wire1.setSCL(27);  // GP27 as SCL
  Wire1.begin();
  
  // Initialize ADS1115
  if (!ads.begin(0x48, &Wire1)) {  // Default address 0x48, use Wire1
    Serial.println("ERROR: Failed to initialize ADS1115!");
    Serial.println("Check wiring and I2C address.");
    while (1) { delay(1000); }  // Halt execution
  }
  
  Serial.println("ADS1115 initialized successfully");
  
  // Set gain for ADS1115 (adjust based on your sensor voltage range)
  // GAIN_TWOTHIRDS = ±6.144V (default)
  // GAIN_ONE = ±4.096V 
  // GAIN_TWO = ±2.048V
  ads.setGain(GAIN_ONE);  // ±4.096V range
  
  Serial.println("Starting sensor readings...");
  Serial.println();
}

void loop() {
  readAllSensors();
  displaySensorData();
  delay(500);  // Read every 500ms
}

void readAllSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    // Read raw ADC value from ADS1115
    sensorRawValues[i] = ads.readADC_SingleEnded(sensorChannels[i]);
    
    // Convert to voltage
    sensorVoltages[i] = ads.computeVolts(sensorRawValues[i]);
    
    // Small delay between readings
    delay(10);
  }
}

void displaySensorData() {
  Serial.println("=== Sensor Readings ===");
  
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorNames[i]);
    Serial.print(" (A");
    Serial.print(sensorChannels[i]);
    Serial.print("): ");
    Serial.print(sensorVoltages[i], 3);  // 3 decimal places
    Serial.print("V (Raw: ");
    Serial.print(sensorRawValues[i]);
    Serial.print(")");
    
    // Add sensor status based on voltage threshold
    if (sensorVoltages[i] > 3.0) {
      Serial.print(" [HIGH]");
    } else if (sensorVoltages[i] > 1.0) {
      Serial.print(" [MID]");
    } else {
      Serial.print(" [LOW]");
    }
    
    Serial.println();
  }
  
  // Display combined sensor state (similar to line tracker pattern)
  uint8_t sensorState = calculateSensorState(2.0);  // 2V threshold
  Serial.print("Combined State (Binary): ");
  Serial.print(sensorState, BIN);
  Serial.print(" (Decimal: ");
  Serial.print(sensorState);
  Serial.println(")");
  Serial.println();
}

uint8_t calculateSensorState(float threshold) {
  uint8_t state = 0;
  
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensorVoltages[i] >= threshold) {
      state |= (1 << i);  // Set bit i if sensor i is above threshold
    }
  }
  
  return state;
}

// Additional utility functions

float getSensorVoltage(uint8_t sensorIndex) {
  if (sensorIndex < NUM_SENSORS) {
    return sensorVoltages[sensorIndex];
  }
  return -1.0;  // Error value
}

int16_t getSensorRawValue(uint8_t sensorIndex) {
  if (sensorIndex < NUM_SENSORS) {
    return sensorRawValues[sensorIndex];
  }
  return -1;  // Error value
}

bool isSensorActive(uint8_t sensorIndex, float threshold) {
  if (sensorIndex < NUM_SENSORS) {
    return sensorVoltages[sensorIndex] >= threshold;
  }
  return false;
}