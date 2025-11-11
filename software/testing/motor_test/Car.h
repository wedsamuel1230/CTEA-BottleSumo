#pragma once

#include <Arduino.h>
#include "Motor.h"

class Car {
public:
  Car() = default;  // Explicitly defaulted constructor
  
  // Initialize both motors with specified pins and PWM frequency
  // Returns false if motor initialization fails
  bool initializeMotors(uint8_t left_pwm_pin, uint8_t left_dir_pin,
                        uint8_t right_pwm_pin, uint8_t right_dir_pin,
                        uint32_t freq = 20000);  // Default 20kHz PWM (silent operation)
  
  // Movement primitives (speed: -100 to +100)
  void forward(float speed);  
  void backward(float speed);
  void turnLeft(float speed);
  void turnRight(float speed);
  void stop();
  
  // Differential drive - individual motor control
  void setMotors(float left_speed, float right_speed);
  
  // Status checks
  bool isInitialized() const;

private:
  Motor leftMotor;
  Motor rightMotor;
};

