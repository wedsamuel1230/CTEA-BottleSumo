// Scalable VL53L0X array manager (up to 5 sensors) with XSHUT control
#pragma once

#ifdef ARDUINO
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#if defined(ARDUINO_ARCH_RP2040)
#include <pico/mutex.h>
#endif
#else
// Minimal fallbacks to satisfy static analysis when not compiling under Arduino
typedef void mutex_t;
class TwoWire {};
struct VL53L0X_RangingMeasurementData_t { uint16_t RangeMilliMeter; uint8_t RangeStatus; };
class Adafruit_VL53L0X {
public:
  bool begin(uint8_t, bool, TwoWire*) {return false; }
  void setMeasurementTimingBudgetMicroSeconds(uint32_t) {}
  void setVcselPulsePeriod(uint8_t, uint8_t) {}
  void rangingTest(VL53L0X_RangingMeasurementData_t*, bool) {}
};
#endif

struct ToFSample {
  uint16_t distanceMm;
  uint8_t status;
  bool valid;
};

class ToFArray {
public:
  // Construct with external Wire bus and optional mutex for arbitration
  ToFArray(TwoWire* wire, mutex_t* wireMutex);

  // Configure sensors: count (<=5), arrays of xshut pins and I2C addresses
  bool configure(uint8_t count, const uint8_t* xshutPins, const uint8_t* i2cAddresses);

  // Set timing parameters for all sensors
  void setTiming(uint32_t timingBudgetUs, uint8_t preRange, uint8_t finalRange);

  // Initialize all sensors with unique addresses; returns number initialized
  uint8_t beginAll();

  // Read all sensors sequentially while holding the bus mutex (if provided)
  void readAll(ToFSample* out, uint16_t minMm = 30, uint16_t maxMm = 1500, uint8_t maxStatus = 2);

  // Status checks (inline for performance)
  uint8_t size() const { return _count; }
  bool isOnline(uint8_t idx) const { return idx < _count ? _online[idx] : false; }

  static constexpr uint8_t MaxSensors = 5;

private:
  TwoWire* _wire;
  mutex_t* _wireMutex;
  Adafruit_VL53L0X _lox[MaxSensors];
  uint8_t _xshut[MaxSensors] = {0};
  uint8_t _addr[MaxSensors] = {0};
  bool _online[MaxSensors] = {false};
  uint8_t _count;
  uint32_t _budgetUs;
  uint8_t _preRange;
  uint8_t _finalRange;
  const uint16_t _resetDelayMs = 50;
  const uint16_t _postResetDelayMs = 20;
};
