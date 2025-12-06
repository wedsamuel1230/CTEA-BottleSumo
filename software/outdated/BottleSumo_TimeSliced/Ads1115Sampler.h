#include <stdint.h>
// Lightweight ADS1115 sampler wrapper to encapsulate init and channel reads
#pragma once

#include <Adafruit_ADS1X15.h>
#include <Wire.h>

class Ads1115Sampler {
public:
  Ads1115Sampler() : _inited(false), _address(0x48), _wire(&Wire) {}

  bool begin(uint8_t i2cAddress, TwoWire* wire, adsGain_t gain, uint8_t rate) {
    _address = i2cAddress;
    _wire = wire ? wire : &Wire;
    _ads.setGain(gain);
    _ads.setDataRate(rate);
    _inited = _ads.begin(_address, _wire);
    return _inited;
  }

  bool isReady() const { return _inited; }

  // Read up to 4 single-ended channels; arrays must have length >= channelCount
  void readAll(int16_t* rawOut, float* voltsOut, uint8_t channelCount = 4) {
    if (!_inited) return;
    if (channelCount > 4) channelCount = 4;
    for (uint8_t i = 0; i < channelCount; ++i) {
      int16_t raw = _ads.readADC_SingleEnded(i);
      if (rawOut) rawOut[i] = raw;
      if (voltsOut) voltsOut[i] = _ads.computeVolts(raw);
    }
  }

private:
  Adafruit_ADS1115 _ads;
  bool _inited;
  uint8_t _address;
  TwoWire* _wire;
};
