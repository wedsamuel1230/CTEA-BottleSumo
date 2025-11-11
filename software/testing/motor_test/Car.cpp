#include <Arduino.h>
#include "Car.h"
#include "Motor.h"

bool Car::initializeMotors(uint8_t left_pwm_pin, uint8_t left_dir_pin,
                           uint8_t right_pwm_pin, uint8_t right_dir_pin,
                           uint32_t freq) {
  leftMotor = Motor(left_pwm_pin, left_dir_pin);
  rightMotor = Motor(right_pwm_pin, right_dir_pin);
  
  bool left_ok = leftMotor.begin(freq);
  bool right_ok = rightMotor.begin(freq);
  
  return left_ok && right_ok;
}

void Car::forward(float speed) {  // Fixed typo: was "foward"
  leftMotor.setDuty(-speed);
  rightMotor.setDuty(speed);
}

void Car::backward(float speed) {
  leftMotor.setDuty(speed);
  rightMotor.setDuty(-speed);
}

void Car::turnLeft(float speed) {
  leftMotor.setDuty(speed);
  rightMotor.setDuty(speed);
}

void Car::turnRight(float speed) {
  leftMotor.setDuty(-speed);
  rightMotor.setDuty(-speed);
}

void Car::stop() {
  leftMotor.stop();
  rightMotor.stop();
}

void Car::setMotors(float left_speed, float right_speed) {
  leftMotor.setDuty(left_speed);
  rightMotor.setDuty(right_speed);
}

bool Car::isInitialized() const {
  return leftMotor.isInitialized() && rightMotor.isInitialized();
}
