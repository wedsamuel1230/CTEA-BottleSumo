/*
 * ADS1115 Advanced Sensor Integration
 * 
 * This example demonstrates advanced usage patterns that can be integrated
 * with the existing sensor systems in the repository (like line trackers).
 * 
 * Features:
 * - Multiple threshold levels
 * - Sensor calibration
 * - Integration-ready functions
 * - Performance optimization
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

// Configuration constants
constexpr uint8_t NUM_SENSORS = 4U;
constexpr float ADS_VREF = 4.096F;
constexpr float DEFAULT_THRESHOLD = 2.0F;
constexpr uint16_t READING_DELAY = 50;  // ms between readings

// Sensor channel mapping
const uint8_t sensorChannels[NUM_SENSORS] = {0, 1, 2, 3};
const uint8_t sensorMasks[NUM_SENSORS] = {0b0001, 0b0010, 0b0100, 0b1000};

// Calibration values (can be adjusted per sensor)
float sensorOffsets[NUM_SENSORS] = {0.0, 0.0, 0.0, 0.0};
float sensorScales[NUM_SENSORS] = {1.0, 1.0, 1.0, 1.0};

// Current sensor data
float currentVoltages[NUM_SENSORS];
int16_t currentRawValues[NUM_SENSORS];
uint32_t lastReadTime = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }
  
  Serial.println(F("ADS1115 Advanced Sensor System"));
  Serial.println(F("=============================="));
  
  // Initialize I2C
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  
  // Initialize ADS1115
  if (!initializeADS1115()) {
    Serial.println(F("FATAL: ADS1115 initialization failed"));
    while (1) { 
      Serial.println(F("Check connections and restart"));
      delay(5000); 
    }
  }
  
  Serial.println(F("System ready - starting sensor monitoring"));
  calibrateSensors();  // Optional calibration routine
}

void loop() {
  // Update sensor readings
  updateSensorReadings();
  
  // Example usage patterns matching repository style
  demonstrateBasicReading();
  demonstrateThresholdDetection();
  demonstrateAdvancedAnalysis();
  
  delay(1000);
}

bool initializeADS1115() {
  if (!ads.begin(0x48, &Wire1)) {
    return false;
  }
  
  // Configure for optimal performance
  ads.setGain(GAIN_ONE);  // ±4.096V range
  ads.setDataRate(RATE_ADS1115_128SPS);  // 128 samples per second
  
  Serial.println(F("ADS1115 configured successfully"));
  return true;
}

void updateSensorReadings() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    // Read raw value
    currentRawValues[i] = ads.readADC_SingleEnded(sensorChannels[i]);
    
    // Convert to voltage with calibration
    float rawVoltage = ads.computeVolts(currentRawValues[i]);
    currentVoltages[i] = (rawVoltage + sensorOffsets[i]) * sensorScales[i];
    
    delay(5);  // Small delay between channel reads
  }
  
  lastReadTime = millis();
}

// Basic reading function (matching repository patterns)
int checkSensorState(float threshold) {
  uint8_t result = 0;
  
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (currentVoltages[i] >= threshold) {
      result |= sensorMasks[i];
    }
    
    Serial.print(F("Sensor"));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(currentVoltages[i], 2);
    Serial.print(F("V "));
  }
  
  Serial.print(F("| State: "));
  Serial.print(result, BIN);
  Serial.print(F(" ("));
  Serial.print(result);
  Serial.println(F(")"));
  
  return result;
}

// Advanced multi-threshold analysis
void analyzeMultiThreshold() {
  const float thresholds[] = {1.0, 2.0, 3.0, 3.5};
  const uint8_t numThresholds = sizeof(thresholds) / sizeof(thresholds[0]);
  
  Serial.println(F("Multi-threshold analysis:"));
  
  for (uint8_t t = 0; t < numThresholds; t++) {
    Serial.print(F("  "));
    Serial.print(thresholds[t], 1);
    Serial.print(F("V: "));
    
    uint8_t state = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (currentVoltages[i] >= thresholds[t]) {
        state |= sensorMasks[i];
      }
    }
    
    Serial.print(state, BIN);
    Serial.print(F(" ("));
    Serial.print(state);
    Serial.println(F(")"));
  }
}

// Sensor statistics
void calculateStatistics() {
  float minVolt = currentVoltages[0];
  float maxVolt = currentVoltages[0];
  float avgVolt = 0;
  
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    avgVolt += currentVoltages[i];
    if (currentVoltages[i] < minVolt) minVolt = currentVoltages[i];
    if (currentVoltages[i] > maxVolt) maxVolt = currentVoltages[i];
  }
  
  avgVolt /= NUM_SENSORS;
  
  Serial.print(F("Stats - Min: "));
  Serial.print(minVolt, 2);
  Serial.print(F("V, Max: "));
  Serial.print(maxVolt, 2);
  Serial.print(F("V, Avg: "));
  Serial.print(avgVolt, 2);
  Serial.println(F("V"));
}

// Calibration routine
void calibrateSensors() {
  Serial.println(F("Starting sensor calibration..."));
  Serial.println(F("Ensure sensors are in reference conditions"));
  delay(3000);
  
  // Take baseline readings
  float baseline[NUM_SENSORS];
  const uint8_t samples = 10;
  
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    baseline[i] = 0;
    for (uint8_t s = 0; s < samples; s++) {
      int16_t raw = ads.readADC_SingleEnded(sensorChannels[i]);
      baseline[i] += ads.computeVolts(raw);
      delay(50);
    }
    baseline[i] /= samples;
    
    Serial.print(F("Sensor "));
    Serial.print(i + 1);
    Serial.print(F(" baseline: "));
    Serial.print(baseline[i], 3);
    Serial.println(F("V"));
  }
  
  Serial.println(F("Calibration complete"));
}

// Demonstration functions
void demonstrateBasicReading() {
  Serial.println(F("=== Basic Reading ==="));
  checkSensorState(DEFAULT_THRESHOLD);
}

void demonstrateThresholdDetection() {
  Serial.println(F("=== Threshold Detection ==="));
  analyzeMultiThreshold();
}

void demonstrateAdvancedAnalysis() {
  Serial.println(F("=== Statistics ==="));
  calculateStatistics();
  Serial.println();
}

// Utility functions for integration with other systems
float getSensorVoltage(uint8_t index) {
  return (index < NUM_SENSORS) ? currentVoltages[index] : -1.0;
}

bool isSensorTriggered(uint8_t index, float threshold) {
  return (index < NUM_SENSORS) ? (currentVoltages[index] >= threshold) : false;
}

uint8_t getActiveSensorMask(float threshold) {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (currentVoltages[i] >= threshold) {
      mask |= sensorMasks[i];
    }
  }
  return mask;
}