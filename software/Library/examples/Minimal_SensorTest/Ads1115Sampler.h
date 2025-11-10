// Lightweight ADS1115 sampler wrapper for blocking 4-channel reads
#pragma once

#include <stdint.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>

class Ads1115Sampler {
public:
  Ads1115Sampler();

  // Initialize ADC with I2C address, wire bus, gain, and sample rate
  bool begin(uint8_t i2cAddress, TwoWire* wire, adsGain_t gain, uint8_t rate);

  // Check if ADC is initialized
  bool isReady() const { return _inited; }

  // Read all 4 single-ended channels (blocking, ~32ms @ 128 SPS)
  // Arrays must have length >= channelCount
  void readAll(int16_t* rawOut, float* voltsOut, uint8_t channelCount = 4);

private:
  Adafruit_ADS1115 _ads;
  bool _inited;
  uint8_t _address;
  TwoWire* _wire;
};
