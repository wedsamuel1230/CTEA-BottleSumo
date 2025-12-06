#include "motor_controller.h"

MotorController::MotorController()
  : _motorLeft(LEFT_PWM_PIN, LEFT_DIR_PIN),
    _motorRight(RIGHT_PWM_PIN, RIGHT_DIR_PIN),
    _leftValue(0),
    _rightValue(0),
    _leftDuty(0.0f),
    _rightDuty(0.0f)
{
}

void MotorController::begin(uint32_t freq) {
  _motorLeft.begin(freq);
  _motorRight.begin(freq);
  stopAll();  // Ensure motors start in safe state
  Serial.println("[MotorController] Initialized L/R motors");
}

float MotorController::mapValueToDuty(int16_t value) {
  // Saturate input to valid range
  if (value > 255) value = 255;
  if (value < -255) value = -255;
  
  // Map [-255, 255] to [-100, 100]
  float duty = (float)value * (100.0f / 255.0f);
  
  // Final saturation (defensive)
  if (duty > 100.0f) duty = 100.0f;
  if (duty < -100.0f) duty = -100.0f;
  
  return duty;
}

void MotorController::setLeft(int16_t value) {
  _leftValue = constrain(value, -255, 255);
  _leftDuty = mapValueToDuty(_leftValue);
  _motorLeft.setDuty(_leftDuty);
}

void MotorController::setRight(int16_t value) {
  _rightValue = constrain(value, -255, 255);
  _rightDuty = mapValueToDuty(_rightValue);
  _motorRight.setDuty(-_rightDuty); // Inverted because right motor is mirrored
}

void MotorController::setBoth(int16_t left, int16_t right) {
  setLeft(left);
  setRight(right);
}

void MotorController::stopAll() {
  _leftValue = 0;
  _rightValue = 0;
  _leftDuty = 0.0f;
  _rightDuty = 0.0f;
  _motorLeft.stop();
  _motorRight.stop();
}
