#include <Arduino.h>
#include "hardware/pwm.h"
#include "Motor.h"

Motor::Motor(uint pwm_pin, uint dir_pin) 
  : _pwm_pin(pwm_pin), _dir_pin(dir_pin), _initialized(false) {
  // Validation deferred to begin() to avoid exceptions in constructor
}

bool Motor::begin(uint32_t freq) {
  // Validate pins (RP2040 has 30 GPIO pins: 0-29)
  if (_pwm_pin > 29 || _dir_pin > 29) {
    return false;  // Invalid pin configuration
  }
  
  // Validate frequency range
  // Absolute hardware limit: freq must result in TOP ≥ 1 (max 62.5 MHz)
  if (freq == 0 || freq > PWM_FREQ_MAX_ABSOLUTE) {
    return false;  // Frequency out of achievable range
  }
  
  // Calculate TOP value to check for practical limits
  uint32_t calculated_top = (RP2040_CLOCK_HZ / freq) - 1;
  
  // Warn if resolution is very low (TOP < 100 means < 1% duty cycle steps)
  // Note: This still allows it, but user should be aware
  // For frequencies > 1.25 MHz, resolution becomes impractical for motor control
  
  // Configure GPIO for PWM function
  gpio_set_function(_pwm_pin, GPIO_FUNC_PWM);
  pinMode(_dir_pin, OUTPUT);

  // Determine PWM slice and channel
  _slice = pwm_gpio_to_slice_num(_pwm_pin);
  _channel = pwm_gpio_to_channel(_pwm_pin);

  // Calculate wrap value (TOP) for target frequency
  // Formula: TOP = (clock / freq) - 1
  _top = (RP2040_CLOCK_HZ / freq) - 1;

  pwm_set_wrap(_slice, _top);
  pwm_set_enabled(_slice, true);
  
  _initialized = true;
  return true;
}

void Motor::setDuty(float speed) {
  if (!_initialized) return;  // Safety check
  
  // Clamp speed to valid range
  speed = clampSpeed(speed);
  
  // Determine direction and absolute duty cycle
  const bool forward = (speed >= 0.0f);
  const float abs_duty = forward ? speed : -speed;
  const float duty_fraction = 1 - abs_duty * 0.01f;  // Convert percentage to 0.0-1.0
  
  // Set direction pin
  digitalWrite(_dir_pin, forward ? HIGH : LOW);
  
  // Calculate and set PWM level
  const uint16_t level = static_cast<uint16_t>(duty_fraction * _top);
  pwm_set_chan_level(_slice, _channel, level);
}

void Motor::stop() {
  if (!_initialized) return;
  
  // Set PWM to _top (100% high time = 0% motor power in inverted logic)
  pwm_set_chan_level(_slice, _channel, _top);
  digitalWrite(_dir_pin, LOW);
}

float Motor::clampSpeed(float speed) {
  if (speed > MAX_SPEED) return MAX_SPEED;
  if (speed < MIN_SPEED) return MIN_SPEED;
  return speed;
}
