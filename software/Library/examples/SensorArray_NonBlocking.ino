/**
 * Non-Blocking Sensor Array Example
 * 
 * Demonstrates simultaneous reading from:
 * - ToFArray (VL53L0X distance sensors)
 * - Ads1115Sampler (analog inputs)
 * 
 * Uses time-sliced non-blocking pattern for optimal performance
 * 
 * Hardware Configuration:
 * - VL53L0X sensors on I2C (Wire)
 * - ADS1115 ADC on I2C (Wire)
 * - RP2040 (Raspberry Pi Pico)
 */

#include <Wire.h>
#include "ToFArray.h"
#include "Ads1115Sampler.h"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// ToF Sensor Configuration (up to 5 sensors)
const uint8_t TOF_COUNT = 3;
const uint8_t tofXshutPins[5] = {16, 17, 18, 19, 20};  // XSHUT pins for sensor reset
const uint8_t tofAddresses[5] = {0x30, 0x31, 0x32, 0x33, 0x34};  // Unique I2C addresses

// ADS1115 Configuration
const uint8_t ADS1115_ADDR = 0x48;  // Default I2C address
const uint8_t ADC_CHANNELS = 4;     // Number of channels to read

// Timing Configuration (milliseconds)
const uint32_t TOF_READ_INTERVAL = 100;    // 10 Hz (100ms between reads)
const uint32_t ADC_READ_INTERVAL = 50;     // 20 Hz (50ms between reads)
const uint32_t SERIAL_PRINT_INTERVAL = 500; // 2 Hz (500ms between prints)

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

ToFArray tofSensors(&Wire, nullptr);  // No mutex for single-core usage
Ads1115Sampler adc;

// Sensor data storage
ToFSample tofReadings[5];
int16_t adcRawValues[4];
float adcVoltages[4];

// Timing state machines
unsigned long lastTofRead = 0;
unsigned long lastAdcRead = 0;
unsigned long lastSerialPrint = 0;

// Status flags
bool tofInitialized = false;
bool adcInitialized = false;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial to stabilize
  
  Serial.println("===========================================");
  Serial.println("  Non-Blocking Sensor Array Example");
  Serial.println("===========================================");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz I2C speed
  Serial.println("✓ I2C initialized at 400 kHz");
  
  // Initialize ToF Array
  Serial.println("\nInitializing ToF sensors...");
  if (tofSensors.configure(TOF_COUNT, tofXshutPins, tofAddresses)) {
    // Optional: Adjust timing for faster reads (reduces accuracy slightly)
    // Default: 33ms budget, 14/10 VCSEL periods
    // Fast mode: 20ms budget, 12/8 VCSEL periods
    tofSensors.setTiming(20000, 12, 8);  // 20ms timing budget
    
    uint8_t onlineCount = tofSensors.beginAll();
    tofInitialized = (onlineCount > 0);
    
    Serial.printf("✓ ToF sensors initialized: %d/%d online\n", onlineCount, TOF_COUNT);
    for (uint8_t i = 0; i < TOF_COUNT; i++) {
      if (tofSensors.isOnline(i)) {
        Serial.printf("  - Sensor %d: ONLINE (addr 0x%02X)\n", i, tofAddresses[i]);
      } else {
        Serial.printf("  - Sensor %d: OFFLINE\n", i);
      }
    }
  } else {
    Serial.println("✗ ToF configuration failed");
  }
  
  // Initialize ADS1115
  Serial.println("\nInitializing ADS1115 ADC...");
  if (adc.begin(ADS1115_ADDR, &Wire, GAIN_TWOTHIRDS, RATE_ADS1115_128SPS)) {
    adcInitialized = adc.isReady();
    Serial.println("✓ ADS1115 initialized");
    Serial.println("  - Gain: ±6.144V");
    Serial.println("  - Rate: 128 SPS");
  } else {
    Serial.println("✗ ADS1115 initialization failed");
  }
  
  Serial.println("\n===========================================");
  Serial.println("Starting non-blocking sensor reads...");
  Serial.println("===========================================\n");
  
  delay(1000);
}

// ============================================================================
// MAIN LOOP - NON-BLOCKING TIME-SLICED EXECUTION
// ============================================================================

void loop() {
  unsigned long currentTime = millis();
  
  // -------------------------------------------------------------------------
  // Task 1: Read ToF Sensors (every 100ms)
  // -------------------------------------------------------------------------
  if (tofInitialized && (currentTime - lastTofRead >= TOF_READ_INTERVAL)) {
    lastTofRead = currentTime;
    
    // Read all ToF sensors at once
    // This is the only "blocking" call, but it's fast (~20-33ms with current settings)
    tofSensors.readAll(tofReadings, 30, 2000, 2);
    
    // Optional: Process readings immediately for time-critical applications
    // processToFData();
  }
  
  // -------------------------------------------------------------------------
  // Task 2: Read ADC Channels (every 50ms)
  // -------------------------------------------------------------------------
  if (adcInitialized && (currentTime - lastAdcRead >= ADC_READ_INTERVAL)) {
    lastAdcRead = currentTime;
    
    // Read all ADC channels
    // Fast operation: ~32ms for 4 channels at 128 SPS
    adc.readAll(adcRawValues, adcVoltages, ADC_CHANNELS);
    
    // Optional: Process ADC data immediately
    // processAdcData();
  }
  
  // -------------------------------------------------------------------------
  // Task 3: Print Sensor Data (every 500ms)
  // -------------------------------------------------------------------------
  if (currentTime - lastSerialPrint >= SERIAL_PRINT_INTERVAL) {
    lastSerialPrint = currentTime;
    
    printSensorData();
  }
  
  // -------------------------------------------------------------------------
  // Task 4: Your application logic here
  // -------------------------------------------------------------------------
  // This is where you'd add robot control logic, decision making, etc.
  // Example: obstacle avoidance, line following, sensor fusion
  
  // Add a small delay to prevent CPU spinning at 100%
  // Adjust or remove based on your performance requirements
  delay(1);  // 1ms delay = ~1000 Hz loop rate
}

// ============================================================================
// SENSOR DATA PROCESSING FUNCTIONS
// ============================================================================

void printSensorData() {
  Serial.println("┌─────────────────────────────────────────────────────────┐");
  Serial.println("│              SENSOR DATA SNAPSHOT                       │");
  Serial.println("├─────────────────────────────────────────────────────────┤");
  
  // Print ToF sensor data
  if (tofInitialized) {
    Serial.println("│ ToF Distance Sensors (VL53L0X):                        │");
    for (uint8_t i = 0; i < TOF_COUNT; i++) {
      if (tofSensors.isOnline(i)) {
        if (tofReadings[i].valid) {
          Serial.printf("│   Sensor %d: %4d mm  [VALID]                          │\n", 
                        i, tofReadings[i].distanceMm);
        } else {
          Serial.printf("│   Sensor %d: ---- mm  [OUT OF RANGE, status=%d]      │\n", 
                        i, tofReadings[i].status);
        }
      } else {
        Serial.printf("│   Sensor %d: OFFLINE                                   │\n", i);
      }
    }
  } else {
    Serial.println("│ ToF Sensors: NOT INITIALIZED                           │");
  }
  
  Serial.println("├─────────────────────────────────────────────────────────┤");
  
  // Print ADC data
  if (adcInitialized) {
    Serial.println("│ ADC Channels (ADS1115):                                │");
    for (uint8_t i = 0; i < ADC_CHANNELS; i++) {
      Serial.printf("│   Ch%d: %6d raw = %6.3f V                            │\n", 
                    i, adcRawValues[i], adcVoltages[i]);
    }
  } else {
    Serial.println("│ ADC: NOT INITIALIZED                                   │");
  }
  
  Serial.println("└─────────────────────────────────────────────────────────┘");
  Serial.println();
}

void processToFData() {
  // Example: Detect obstacles in front
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    if (tofReadings[i].valid && tofReadings[i].distanceMm < 200) {
      // Obstacle detected within 20cm
      Serial.printf("[WARNING] Obstacle detected on sensor %d: %d mm\n", 
                    i, tofReadings[i].distanceMm);
    }
  }
}

void processAdcData() {
  // Example: Monitor battery voltage (assuming channel 0 is battery)
  const float BATTERY_LOW_THRESHOLD = 3.3;  // Example threshold
  
  if (adcVoltages[0] < BATTERY_LOW_THRESHOLD) {
    Serial.printf("[WARNING] Low battery: %.2fV\n", adcVoltages[0]);
  }
}

// ============================================================================
// ADVANCED: SENSOR FUSION EXAMPLE
// ============================================================================

struct RobotState {
  bool obstacleAhead;
  bool obstacleLeft;
  bool obstacleRight;
  float batteryVoltage;
  float frontDistance;
};

RobotState getSensorFusedState() {
  RobotState state = {false, false, false, 0.0f, 0.0f};
  
  // Process ToF sensors (assuming 3 sensors: left, center, right)
  if (tofInitialized && TOF_COUNT >= 3) {
    if (tofReadings[1].valid) {
      state.frontDistance = tofReadings[1].distanceMm;
      state.obstacleAhead = (tofReadings[1].distanceMm < 300);  // 30cm threshold
    }
    
    if (tofReadings[0].valid) {
      state.obstacleLeft = (tofReadings[0].distanceMm < 200);   // 20cm threshold
    }
    
    if (tofReadings[2].valid) {
      state.obstacleRight = (tofReadings[2].distanceMm < 200);  // 20cm threshold
    }
  }
  
  // Process ADC (assuming channel 0 is battery monitor)
  if (adcInitialized) {
    state.batteryVoltage = adcVoltages[0];
  }
  
  return state;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void printSystemStatus() {
  Serial.println("\n=== SYSTEM STATUS ===");
  Serial.printf("ToF Array: %s (%d/%d online)\n", 
                tofInitialized ? "OK" : "FAILED", 
                tofInitialized ? tofSensors.size() : 0, 
                TOF_COUNT);
  Serial.printf("ADS1115:   %s\n", adcInitialized ? "OK" : "FAILED");
  Serial.printf("Loop rate: ~%d Hz\n", 1000 / max(TOF_READ_INTERVAL, ADC_READ_INTERVAL));
  Serial.println("====================\n");
}
