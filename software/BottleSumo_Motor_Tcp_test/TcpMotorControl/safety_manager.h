#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <Arduino.h>

// Safety manager enforces sensor-based retreat patterns and emergency stop
// Per contracts: pattern [A3 A2 A1 A0] maps to specific retreat behaviors
class SafetyManager {
public:
  SafetyManager();
  
  // Update safety state with sensor pattern
  // pattern: 4-bit value [A3 A2 A1 A0] where bit=1 means edge detected
  void update(uint8_t sensorPattern);
  
  // Trigger software emergency stop
  void triggerEstop();
  
  // Check if system is in safe state
  bool isSafe() const { return !_estop && !_latched; }
  
  // Check if emergency stop is active
  bool isEstop() const { return _estop; }
  
  // Check if unsafe latched
  bool isLatched() const { return _latched; }
  
  // Attempt to clear latch (requires safe sensor state and dwell time)
  bool tryClearLatch();
  
  // Get retreat motor values (used when sensors trigger)
  // Returns false if motion should be blocked
  bool getRetreatMotion(int16_t& left, int16_t& right) const;
  
  // Validate if a commanded motion is safe given current sensor state
  // Returns true if safe, false if would drive toward edge
  bool validateMotion(int16_t cmdLeft, int16_t cmdRight) const;

private:
  bool _estop;              // Emergency stop active
  bool _latched;            // Unsafe state latched
  uint8_t _currentPattern;  // Current sensor pattern
  unsigned long _latchTimeMs; // When latch was set
  
  static const uint32_t LATCH_DWELL_MS = 500; // Time before latch can clear
  
  // Retreat patterns per contracts (L, R duty in range -100..100)
  // We'll scale these to [-255, 255] for motor controller
  struct RetreatVector {
    int16_t left;
    int16_t right;
  };
  
  // Get retreat vector for sensor pattern
  RetreatVector getRetreatVector(uint8_t pattern) const;
  
  // Check if pattern indicates emergency (>=3 sensors)
  bool isEmergencyPattern(uint8_t pattern) const;
};

#endif // SAFETY_MANAGER_H
