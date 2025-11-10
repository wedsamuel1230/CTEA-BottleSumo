#include <Arduino.h>
#include "Ads1115Sampler.h"

Ads1115Sampler::Ads1115Sampler() 
  : _inited(false), _address(0x48), _wire(&Wire), _currentChannel(0), _lastRaw(0) {
  // Constructor with initializer list
}

bool Ads1115Sampler::begin(uint8_t i2cAddress, TwoWire* wire, adsGain_t gain, uint8_t rate) {
  _address = i2cAddress;
  _wire = wire ? wire : &Wire;
  _ads.setGain(gain);
  _ads.setDataRate(rate);
  _inited = _ads.begin(_address, _wire);
  
  // Start continuous conversion on channel 0
  if (_inited) {
    _ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/true);
    _currentChannel = 0;
    _conversionStartTime = millis();
  }
  
  return _inited;
}

// ============================================================================
// BLOCKING API (DEPRECATED - kept for compatibility)
// ============================================================================

void Ads1115Sampler::readAll(int16_t* rawOut, float* voltsOut, uint8_t channelCount) {
  if (!_inited) return;
  if (channelCount > 4) channelCount = 4;
  for (uint8_t i = 0; i < channelCount; ++i) {
    int16_t raw = _ads.readADC_SingleEnded(i);
    if (rawOut) rawOut[i] = raw;
    if (voltsOut) voltsOut[i] = _ads.computeVolts(raw);
  }
}

// ============================================================================
// NON-BLOCKING API
// ============================================================================

void Ads1115Sampler::startConversion(uint8_t channel) {
  if (!_inited || channel > 3) return;
  
  // Switch to new channel and trigger read (continuous mode will keep reading)
  _currentChannel = channel;
  _ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0 | (channel & 0x03), /*continuous=*/true);
  _conversionStartTime = millis();  // Track conversion start time
}

bool Ads1115Sampler::poll() {
  if (!_inited) return false;
  
  // Check if conversion is complete OR if timeout reached (safety net for stuck conversion)
  unsigned long elapsed = millis() - _conversionStartTime;
  if (_ads.conversionComplete() || elapsed >= _conversionTimeoutMs) {
    _lastRaw = _ads.getLastConversionResults();
    return true;
  }
  
  return false;
}

float Ads1115Sampler::getLastResultVolts() const {
  // Note: computeVolts() is non-const in Adafruit library, so we cast away const
  return const_cast<Adafruit_ADS1115&>(_ads).computeVolts(_lastRaw);
}

