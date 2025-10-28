#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "Motor.h"

// Motor controller wrapper for dual motors (Left/Right)
// Maps command values [-255,255] to Motor duty [-100,100]
class MotorController {
public:
  MotorController();
  
  // Initialize both motors with PWM frequency
  void begin(uint32_t freq = 20000);
  
  // Set motor speeds: value in [-255, 255]
  // Positive = forward, Negative = reverse, 0 = stop
  // Values are saturated to valid range
  void setLeft(int16_t value);
  void setRight(int16_t value);
  void setBoth(int16_t left, int16_t right);
  
  // Emergency stop - both motors off immediately
  void stopAll();
  
  // Get current commanded values
  int16_t getLeftValue() const { return _leftValue; }
  int16_t getRightValue() const { return _rightValue; }
  
  // Get applied duty cycles [-100, 100]
  float getLeftDuty() const { return _leftDuty; }
  float getRightDuty() const { return _rightDuty; }

private:
  Motor _motorLeft;
  Motor _motorRight;
  
  int16_t _leftValue;   // Current commanded value [-255, 255]
  int16_t _rightValue;
  float _leftDuty;      // Applied duty [-100, 100]
  float _rightDuty;
  
  // Convert value [-255,255] to duty [-100,100] with saturation
  float mapValueToDuty(int16_t value);
  
  // Pin definitions (per plan.md GPIO map)
  static const uint8_t LEFT_PWM_PIN = 11;   // GP11
  static const uint8_t LEFT_DIR_PIN = 10;   // GP10
  static const uint8_t RIGHT_PWM_PIN = 14;  // GP14
  static const uint8_t RIGHT_DIR_PIN = 13;  // GP13
};

#endif // MOTOR_CONTROLLER_H
