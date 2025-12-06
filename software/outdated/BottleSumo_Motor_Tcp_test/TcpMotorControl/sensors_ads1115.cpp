#include "sensors_ads1115.h"

SensorsADS1115::SensorsADS1115() {
  // Initialize arrays
  for (int i = 0; i < 4; i++) {
    _rawValues[i] = 0;
    _thresholds[i] = DEFAULT_THRESHOLD;
    _flags[i] = false;
    _debounceCounts[i] = 0;
  }
}

bool SensorsADS1115::begin() {
  // Initialize Wire1 on GP26/GP27
  Wire1.setSDA(I2C_SDA_PIN);
  Wire1.setSCL(I2C_SCL_PIN);
  Wire1.begin();
  
  // Initialize ADS1115
  if (!_ads.begin(I2C_ADDRESS, &Wire1)) {
    Serial.println("[Sensors] ERROR: Failed to initialize ADS1115");
    return false;
  }
  
  // Set gain for better resolution (adjust based on QRE1113 voltage range)
  // GAIN_ONE = +/-4.096V (default)
  _ads.setGain(GAIN_ONE);
  
  // Set data rate for fast sampling (860 SPS)
  _ads.setDataRate(RATE_ADS1115_860SPS);
  
  Serial.println("[Sensors] ADS1115 initialized on Wire1");
  return true;
}

void SensorsADS1115::update() {
  // Read all 4 channels
  for (int i = 0; i < 4; i++) {
    _rawValues[i] = _ads.readADC_SingleEnded(i);
    updateFlag(i);
  }
}

void SensorsADS1115::updateFlag(uint8_t channel) {
  if (channel >= 4) return;
  
  // Compare raw value to threshold
  // QRE1113: Lower value = darker surface (edge detected)
  bool edgeDetected = (_rawValues[channel] < _thresholds[channel]);
  
  // Simple debounce: require consecutive readings
  if (edgeDetected) {
    if (_debounceCounts[channel] < DEBOUNCE_THRESHOLD) {
      _debounceCounts[channel]++;
    }
    if (_debounceCounts[channel] >= DEBOUNCE_THRESHOLD) {
      _flags[channel] = true;
    }
  } else {
    _debounceCounts[channel] = 0;
    _flags[channel] = false;
  }
}

void SensorsADS1115::setThreshold(uint8_t channel, uint16_t threshold) {
  if (channel < 4) {
    _thresholds[channel] = threshold;
  }
}

void SensorsADS1115::setAllThresholds(uint16_t t0, uint16_t t1, uint16_t t2, uint16_t t3) {
  _thresholds[0] = t0;
  _thresholds[1] = t1;
  _thresholds[2] = t2;
  _thresholds[3] = t3;
}

bool SensorsADS1115::calibrateAuto(uint16_t samples) {
  Serial.println("[Sensors] Starting auto-calibration...");
  Serial.print("[Sensors] Place robot on WHITE surface, sampling ");
  Serial.print(samples);
  Serial.println(" readings...");
  
  // Accumulate readings for white surface
  uint32_t sums[4] = {0, 0, 0, 0};
  
  for (uint16_t s = 0; s < samples; s++) {
    for (int i = 0; i < 4; i++) {
      sums[i] += _ads.readADC_SingleEnded(i);
    }
    delay(5); // Small delay between samples
  }
  
  // Calculate averages
  uint16_t whiteValues[4];
  for (int i = 0; i < 4; i++) {
    whiteValues[i] = sums[i] / samples;
  }
  
  // Set thresholds at 70% of white value (edge/dark is lower)
  // This provides margin while detecting edges
  for (int i = 0; i < 4; i++) {
    _thresholds[i] = (uint16_t)(whiteValues[i] * 0.70f);
    Serial.print("[Sensors] Ch");
    Serial.print(i);
    Serial.print(" White=");
    Serial.print(whiteValues[i]);
    Serial.print(" Threshold=");
    Serial.println(_thresholds[i]);
  }
  
  Serial.println("[Sensors] Calibration complete");
  return true;
}

uint8_t SensorsADS1115::getPattern() const {
  // Build 4-bit pattern [A3 A2 A1 A0]
  uint8_t pattern = 0;
  for (int i = 0; i < 4; i++) {
    if (_flags[i]) {
      pattern |= (1 << i);
    }
  }
  return pattern;
}

uint8_t SensorsADS1115::getTriggeredCount() const {
  uint8_t count = 0;
  for (int i = 0; i < 4; i++) {
    if (_flags[i]) count++;
  }
  return count;
}
