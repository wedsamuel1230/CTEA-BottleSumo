/**************************************************************************
 ADS1115 Library Example for Raspberry Pi Pico
 
 This sketch demonstrates using the popular Adafruit ADS1X15 library
 for easier ADS1115 integration with existing BottleSumo code.
 
 Library Installation:
 1. Open Arduino IDE
 2. Go to Tools -> Manage Libraries
 3. Search for "Adafruit ADS1X15"
 4. Install the library by Adafruit
 
 This provides a simpler API compared to direct register manipulation
 while maintaining compatibility with the existing I2C setup.
 
 Features:
 - Simplified API calls
 - Automatic gain management
 - Built-in voltage conversion
 - Compatible with existing Wire1 setup
 **************************************************************************/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// Create ADS1115 object
Adafruit_ADS1115 ads;

// Line tracking configuration (same as existing code)
constexpr uint8_t NUM_SENSORS = 4;
const char* sensorNames[NUM_SENSORS] = {"Left", "L-Center", "R-Center", "Right"};
const uint8_t sensorMask[NUM_SENSORS] = {0b0001, 0b0010, 0b0100, 0b1000};

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for serial monitor
  
  // Initialize I2C using the same pattern as other sketches in this repo
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  
  Serial.println("ADS1115 Library Example");
  Serial.println("=======================");
  Serial.println("I2C initialized: SDA=GP26, SCL=GP27");
  
  // Initialize ADS1115 with custom I2C interface
  if (!ads.begin(0x48, &Wire1)) {
    Serial.println("ERROR: ADS1115 not found. Check connections.");
    while(1);
  }
  
  Serial.println("ADS1115 initialized successfully!");
  
  // Set gain for appropriate voltage range
  // GAIN_TWOTHIRDS: +/- 6.144V  1 bit = 3mV (default)
  // GAIN_ONE:       +/- 4.096V  1 bit = 2mV
  // GAIN_TWO:       +/- 2.048V  1 bit = 1mV
  // GAIN_FOUR:      +/- 1.024V  1 bit = 0.5mV
  // GAIN_EIGHT:     +/- 0.512V  1 bit = 0.25mV
  // GAIN_SIXTEEN:   +/- 0.256V  1 bit = 0.125mV
  ads.setGain(GAIN_ONE); // +/- 4.096V range for 3.3V/5V sensors
  
  Serial.println("Gain set to +/- 4.096V range");
  Serial.println("\nStarting sensor readings...\n");
}

void loop() {
  // Method 1: Simple voltage reading for all channels
  Serial.println("=== Method 1: Direct Voltage Reading ===");
  readAllChannelsSimple();
  
  delay(1000);
  
  // Method 2: Line tracking integration (compatible with existing code)
  Serial.println("\n=== Method 2: Line Tracking Integration ===");
  int result = checkLineTrackerADS1115Library(3.0);
  Serial.print("Line Tracker Result: ");
  Serial.print(result, BIN);
  Serial.print(" (");
  Serial.print(result);
  Serial.println(")");
  
  Serial.println("\n" + String('-', 50) + "\n");
  delay(2000);
}

// Simple method to read all 4 channels and display voltages
void readAllChannelsSimple() {
  Serial.println("Channel | Raw Value | Voltage");
  Serial.println("--------|-----------|--------");
  
  for (int i = 0; i < 4; i++) {
    // Read raw ADC value
    int16_t rawValue = ads.readADC_SingleEnded(i);
    
    // Convert to voltage using library function
    float voltage = ads.computeVolts(rawValue);
    
    Serial.print("   ");
    Serial.print(i);
    Serial.print("    |   ");
    Serial.print(rawValue);
    Serial.print("   | ");
    Serial.print(voltage, 3);
    Serial.println("V");
  }
}

// Line tracking function using ADS1115 library (compatible with existing code style)
int checkLineTrackerADS1115Library(float thresholdV) {
  uint8_t result = 0;
  
  Serial.println("Sensor    | Voltage | Status");
  Serial.println("----------|---------|--------");
  
  for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
    // Read voltage directly using library
    int16_t rawValue = ads.readADC_SingleEnded(i);
    float voltage = ads.computeVolts(rawValue);
    
    bool outOfBounds = (voltage >= thresholdV);
    if (outOfBounds) {
      result |= sensorMask[i];
    }
    
    Serial.print(sensorNames[i]);
    // Pad sensor name to 9 characters
    for (int j = strlen(sensorNames[i]); j < 9; j++) {
      Serial.print(" ");
    }
    Serial.print("| ");
    Serial.print(voltage, 2);
    Serial.print("V  | ");
    Serial.println(outOfBounds ? "OUT" : "OK");
  }
  
  return result;
}

// Alternative differential reading example (useful for noise reduction)
void readDifferentialExample() {
  Serial.println("\n=== Differential Reading Example ===");
  
  // Read differential between channels (AIN0-AIN1 and AIN2-AIN3)
  int16_t diff01 = ads.readADC_Differential_0_1();
  int16_t diff23 = ads.readADC_Differential_2_3();
  
  float voltage01 = ads.computeVolts(diff01);
  float voltage23 = ads.computeVolts(diff23);
  
  Serial.print("Differential AIN0-AIN1: ");
  Serial.print(voltage01, 3);
  Serial.println("V");
  
  Serial.print("Differential AIN2-AIN3: ");
  Serial.print(voltage23, 3);
  Serial.println("V");
}

// Advanced example: Continuous reading with alert
void setupContinuousReading() {
  // Example of setting up continuous reading mode
  // This can be useful for real-time line tracking
  
  // Set up comparator for channel 0
  // Alert when voltage goes above 3.0V
  ads.startComparator_SingleEnded(0, 1000); // ~3.0V at current gain
  
  Serial.println("Continuous reading setup complete");
  Serial.println("Alert will trigger when AIN0 > 3.0V");
}

// Function to check alert status
bool checkAlert() {
  // Read the config register to check alert status
  // This is advanced usage - basic polling is usually sufficient
  return digitalRead(2); // If ALERT pin connected to GPIO2
}