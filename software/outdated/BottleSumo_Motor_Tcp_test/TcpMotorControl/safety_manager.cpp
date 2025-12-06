#include "safety_manager.h"

SafetyManager::SafetyManager()
  : _estop(false),
    _latched(false),
    _currentPattern(0),
    _latchTimeMs(0)
{
}

void SafetyManager::update(uint8_t sensorPattern) {
  _currentPattern = sensorPattern;
  
  // Check for emergency pattern (>=3 sensors)
  if (isEmergencyPattern(sensorPattern)) {
    Serial.println("[Safety] EMERGENCY: >=3 sensors triggered!");
    _estop = true;
    _latched = true;
    _latchTimeMs = millis();
    return;
  }
  
  // If any sensor is triggered, latch unsafe
  if (sensorPattern != 0) {
    if (!_latched) {
      Serial.print("[Safety] Edge detected, pattern: 0x");
      Serial.println(sensorPattern, HEX);
      _latched = true;
      _latchTimeMs = millis();
    }
  }
}

void SafetyManager::triggerEstop() {
  Serial.println("[Safety] ESTOP triggered by command");
  _estop = true;
  _latched = true;
  _latchTimeMs = millis();
}

bool SafetyManager::tryClearLatch() {
  // Cannot clear estop via this method (requires manual intervention)
  if (_estop) {
    return false;
  }
  
  // Check if sensors are clear and dwell time has passed
  if (_currentPattern == 0 && _latched) {
    unsigned long elapsed = millis() - _latchTimeMs;
    if (elapsed >= LATCH_DWELL_MS) {
      Serial.println("[Safety] Latch cleared");
      _latched = false;
      return true;
    }
  }
  
  return false;
}

SafetyManager::RetreatVector SafetyManager::getRetreatVector(uint8_t pattern) const {
  // Map patterns to retreat vectors per contracts/jsonl-commands.md
  // Scale from [-100, 100] duty to [-255, 255] command values
  // Multiplier: 255/100 = 2.55
  
  RetreatVector vec;
  
  switch (pattern) {
    case 0x1: // A0 only (front-left)
      vec.left = -204;   // -80 * 2.55
      vec.right = -102;  // -40 * 2.55
      break;
    case 0x2: // A1 only (front-right)
      vec.left = -102;
      vec.right = -204;
      break;
    case 0x4: // A2 only (rear-left)
      vec.left = 204;
      vec.right = 102;
      break;
    case 0x8: // A3 only (rear-right)
      vec.left = 102;
      vec.right = 204;
      break;
    case 0x3: // A0+A1 (front)
      vec.left = -204;
      vec.right = -204;
      break;
    case 0xC: // A2+A3 (rear)
      vec.left = 204;
      vec.right = 204;
      break;
    case 0x5: // A0+A2 (left side)
      vec.left = 153;    // 60 * 2.55
      vec.right = -153;
      break;
    case 0xA: // A1+A3 (right side)
      vec.left = -153;
      vec.right = 153;
      break;
    case 0x9: // A0+A3 (diagonal)
      vec.left = 204;
      vec.right = 102;
      break;
    case 0x6: // A1+A2 (diagonal)
      vec.left = 102;
      vec.right = 204;
      break;
    default:
      // Unknown or emergency pattern: stop
      vec.left = 0;
      vec.right = 0;
      break;
  }
  
  return vec;
}

bool SafetyManager::getRetreatMotion(int16_t& left, int16_t& right) const {
  if (_estop) {
    // Estop: full stop
    left = 0;
    right = 0;
    return false;
  }
  
  if (_currentPattern == 0) {
    // No sensors triggered, no retreat needed
    return true;
  }
  
  // Get retreat vector
  RetreatVector vec = getRetreatVector(_currentPattern);
  left = vec.left;
  right = vec.right;
  
  return true;
}

bool SafetyManager::validateMotion(int16_t cmdLeft, int16_t cmdRight) const {
  // If estop, block all motion
  if (_estop) {
    return false;
  }
  
  // If no sensors, allow any motion
  if (_currentPattern == 0) {
    return true;
  }
  
  // Check if commanded motion would drive toward detected edge
  // Simple heuristic: if retreat vector opposes command, block
  RetreatVector retreat = getRetreatVector(_currentPattern);
  
  // If retreat suggests backward and command is forward, block
  // (and vice versa)
  if ((retreat.left < 0 && cmdLeft > 50) || (retreat.left > 0 && cmdLeft < -50)) {
    Serial.println("[Safety] Motion blocked: would drive toward edge");
    return false;
  }
  if ((retreat.right < 0 && cmdRight > 50) || (retreat.right > 0 && cmdRight < -50)) {
    Serial.println("[Safety] Motion blocked: would drive toward edge");
    return false;
  }
  
  return true;
}

bool SafetyManager::isEmergencyPattern(uint8_t pattern) const {
  // Count bits set in pattern
  uint8_t count = 0;
  for (int i = 0; i < 4; i++) {
    if (pattern & (1 << i)) count++;
  }
  return (count >= 3);
}
