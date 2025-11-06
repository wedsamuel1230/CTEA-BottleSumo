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

  // Read up to 4 single-ended channels; arrays must have length >= channelCount
  void readAll(int16_t* rawOut, float* voltsOut, uint8_t channelCount = 4);

private:
  Adafruit_ADS1115 _ads;
  bool _inited;
  uint8_t _address;
  TwoWire* _wire;
};
