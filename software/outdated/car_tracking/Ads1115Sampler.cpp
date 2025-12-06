#include <Arduino.h>
#include "Ads1115Sampler.h"

Ads1115Sampler::Ads1115Sampler() 
  : _inited(false), _address(0x48), _wire(&Wire) {
  // Constructor with initializer list
}

bool Ads1115Sampler::begin(uint8_t i2cAddress, TwoWire* wire, adsGain_t gain, uint8_t rate) {
  _address = i2cAddress;
  _wire = wire ? wire : &Wire;
  _ads.setGain(gain);
  _ads.setDataRate(rate);
  _inited = _ads.begin(_address, _wire);
  return _inited;
}

void Ads1115Sampler::readAll(int16_t* rawOut, float* voltsOut, uint8_t channelCount) {
  if (!_inited) return;
  if (channelCount > 4) channelCount = 4;
  
  for (uint8_t i = 0; i < channelCount; ++i) {
    int16_t raw = _ads.readADC_SingleEnded(i);
    if (rawOut) rawOut[i] = raw;
    if (voltsOut) voltsOut[i] = _ads.computeVolts(raw);
  }
}

