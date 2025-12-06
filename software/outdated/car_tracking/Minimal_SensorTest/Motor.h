#pragma once

#include <Arduino.h>

class Motor {
public:
  // Hardware constraints for RP2040
  static constexpr uint32_t RP2040_CLOCK_HZ = 125000000UL;
  static constexpr float MAX_SPEED = 100.0f;
  static constexpr float MIN_SPEED = -100.0f;
  
  // PWM Frequency Limits (Hz)
  // TOP register is 16-bit (0-65535), Formula: TOP = (clock / freq) - 1
  static constexpr uint32_t PWM_FREQ_MIN = 1908;           // TOP = 65535 (max counter)
  static constexpr uint32_t PWM_FREQ_MAX_ABSOLUTE = 62500000;  // TOP = 1 (no resolution!)
  static constexpr uint32_t PWM_FREQ_MAX_PRACTICAL = 1250000;  // TOP = 100 (1% resolution)
  static constexpr uint32_t PWM_FREQ_MAX_RECOMMENDED = 488281; // TOP = 256 (0.4% resolution)
  static constexpr uint32_t PWM_FREQ_MOTOR_OPTIMAL = 20000;    // 20 kHz (silent, efficient)

  Motor() = default;  // Default constructor for array/member initialization
  Motor(uint pwm_pin, uint dir_pin);
  
  // Initialize PWM with specified frequency (Hz)
  // Returns false if pin configuration is invalid
  bool begin(uint32_t freq);
  
  // Set motor speed: -100 (full reverse) to +100 (full forward)
  // Values outside range are clamped
  void setDuty(float speed);
  
  // Emergency stop - sets duty to 0 and direction LOW
  void stop();
  
  // Check if motor is initialized
  bool isInitialized() const { return _initialized; }

private:
  uint _pwm_pin = 0;  
  uint _dir_pin = 0;
  uint _slice = 0;
  uint _channel = 0;
  uint16_t _top = 0;
  bool _initialized = false;
  
  // Clamp speed to valid range
  static float clampSpeed(float speed);
};
