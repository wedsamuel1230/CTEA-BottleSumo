#include <Arduino.h>
#include "ToFArray.h"

ToFArray::ToFArray(TwoWire* wire, mutex_t* wireMutex)
  : _wire(wire ? wire : &Wire), _wireMutex(wireMutex), _count(0), _budgetUs(33000), _preRange(14), _finalRange(10) {
  // Constructor with initializer list
}

bool ToFArray::configure(uint8_t count, const uint8_t* xshutPins, const uint8_t* i2cAddresses) {
  if (count == 0 || count > MaxSensors) return false;
  _count = count;
  for (uint8_t i = 0; i < _count; ++i) {
    _xshut[i] = xshutPins[i];
    _addr[i] = i2cAddresses[i];
    pinMode(_xshut[i], OUTPUT);
    digitalWrite(_xshut[i], LOW);
  }
  delay(_resetDelayMs);
  return true;
}

void ToFArray::setTiming(uint32_t timingBudgetUs, uint8_t preRange, uint8_t finalRange) {
  _budgetUs = timingBudgetUs;
  _preRange = preRange;
  _finalRange = finalRange;
}

uint8_t ToFArray::beginAll() {
  uint8_t ok = 0;
  for (uint8_t i = 0; i < _count; ++i) {
    digitalWrite(_xshut[i], HIGH);
    delay(_postResetDelayMs);
    if (!_lox[i].begin(_addr[i], false, _wire)) {
      _online[i] = false;
      continue;
    }
    _lox[i].setMeasurementTimingBudgetMicroSeconds(_budgetUs);
    _lox[i].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, _preRange);
    _lox[i].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, _finalRange);
    _online[i] = true;
    ok++;
  }
  return ok;
}

void ToFArray::readAll(ToFSample* out, uint16_t minMm, uint16_t maxMm, uint8_t maxStatus) {
  if (_wireMutex) mutex_enter_blocking(_wireMutex);
  for (uint8_t i = 0; i < _count; ++i) {
    if (!_online[i]) {
      out[i] = {0, 0xFF, false};
      continue;
    }
    VL53L0X_RangingMeasurementData_t data;
    _lox[i].rangingTest(&data, false);
    bool valid = (data.RangeStatus <= maxStatus) && (data.RangeMilliMeter >= minMm) && (data.RangeMilliMeter < maxMm);
    out[i].distanceMm = valid ? data.RangeMilliMeter : 0;
    out[i].status = data.RangeStatus;
    out[i].valid = valid;
  }
  if (_wireMutex) mutex_exit(_wireMutex);
}