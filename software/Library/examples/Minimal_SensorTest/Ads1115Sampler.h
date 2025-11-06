// Lightweight ADS1115 sampler wrapper to encapsulate init and channel reads
#pragma once

#include <stdint.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>

class Ads1115Sampler {
public:
  Ads1115Sampler();

  bool begin(uint8_t i2cAddress, TwoWire* wire, adsGain_t gain, uint8_t rate);

  // Trivial getter (inline for performance)
  bool isReady() const { return _inited; }

  // ========== BLOCKING API (DEPRECATED) ==========
  // Read up to 4 single-ended channels; arrays must have length >= channelCount
  // WARNING: Blocks for ~30-32ms. Use non-blocking API for better performance.
  void readAll(int16_t* rawOut, float* voltsOut, uint8_t channelCount = 4);

  // ========== NON-BLOCKING API ==========
  // Start conversion on specified channel (0-3)
  void startConversion(uint8_t channel);
  
  // Poll for conversion ready and fetch last result
  // Returns: true if new data available, false if still converting
  // Use getLastResult() to retrieve the value
  bool poll();
  
  // Get last conversion result (raw ADC value)
  int16_t getLastResult() const { return _lastRaw; }
  
  // Get last conversion result as voltage
  float getLastResultVolts() const;

private:
  Adafruit_ADS1115 _ads;
  bool _inited;
  uint8_t _address;
  TwoWire* _wire;
  uint8_t _currentChannel;
  int16_t _lastRaw;
};
