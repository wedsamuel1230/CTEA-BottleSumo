#ifndef SENSORS_ADS1115_H
#define SENSORS_ADS1115_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ADS1115 sensor manager for 4x QRE1113 edge sensors
// Channels: A0=front-left, A1=front-right, A2=rear-left, A3=rear-right
class SensorsADS1115 {
public:
  SensorsADS1115();
  
  // Initialize ADS1115 on Wire1 (GP26=SDA, GP27=SCL)
  bool begin();
  
  // Read all 4 channels and update flags
  void update();
  
  // Get raw ADC values [0-32767 for ADS1115 16-bit]
  const uint16_t* getRawValues() const { return _rawValues; }
  uint16_t getRaw(uint8_t channel) const { return _rawValues[channel]; }
  
  // Get edge detection flags (true = edge/dark detected)
  const bool* getFlags() const { return _flags; }
  bool getFlag(uint8_t channel) const { return _flags[channel]; }
  
  // Calibration: set thresholds manually
  void setThreshold(uint8_t channel, uint16_t threshold);
  void setAllThresholds(uint16_t t0, uint16_t t1, uint16_t t2, uint16_t t3);
  
  // Auto-calibration: sample white surface and set thresholds
  // Returns true on success
  bool calibrateAuto(uint16_t samples = 64);
  
  // Get thresholds
  const uint16_t* getThresholds() const { return _thresholds; }
  
  // Get sensor pattern as 4-bit value [A3 A2 A1 A0]
  uint8_t getPattern() const;
  
  // Count number of sensors triggered
  uint8_t getTriggeredCount() const;

private:
  Adafruit_ADS1115 _ads;
  
  uint16_t _rawValues[4];
  uint16_t _thresholds[4];
  bool _flags[4];
  
  // Debounce state
  uint16_t _debounceCounts[4];
  static const uint8_t DEBOUNCE_THRESHOLD = 3; // Consecutive readings required
  
  // I2C pins for Wire1
  static const uint8_t I2C_SDA_PIN = 26; // GP26
  static const uint8_t I2C_SCL_PIN = 27; // GP27
  static const uint8_t I2C_ADDRESS = 0x48; // ADS1115 default
  
  // Default threshold (mid-range, to be calibrated)
  static const uint16_t DEFAULT_THRESHOLD = 16000;
  
  // Update flag for a channel with debouncing
  void updateFlag(uint8_t channel);
};

#endif // SENSORS_ADS1115_H
