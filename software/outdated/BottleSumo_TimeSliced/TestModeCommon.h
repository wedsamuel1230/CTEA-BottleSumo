/**
 * TestModeCommon.h - Shared Test Mode Definitions
 * 
 * Common enums, structures, and utilities for all test modes.
 * Designed to be modular and portable to TimeSliced version.
 * 
 * Features:
 * - Mode enumeration (AUTO, TEST_MOTOR, TEST_SENSOR, CALIBRATE)
 * - Safety timeout management
 * - Test state tracking
 * 
 * Author: CTEA-BottleSumo Project
 * Date: 2025-10-15
 * License: MIT
 */

#ifndef TEST_MODE_COMMON_H
#define TEST_MODE_COMMON_H

#include <Arduino.h>

// ========== Test Mode Enumeration ==========

enum class TestMode : uint8_t {
  AUTO = 0,           // Autonomous mode (normal operation)
  TEST_MOTOR = 1,     // Motor PWM testing
  TEST_SENSOR = 2,    // Sensor individual testing
  CALIBRATE_IR = 3,   // IR sensor calibration
  CALIBRATE_TOF = 4   // ToF sensor calibration
};

// ========== Test State Structure ==========

struct TestModeState {
  TestMode mode;                    // Current test mode
  unsigned long last_command_ms;    // Last command timestamp (for timeout)
  unsigned long mode_enter_ms;      // Mode entry timestamp
  bool safety_timeout_enabled;      // Enable safety timeout (true = stop on timeout)
  uint32_t timeout_duration_ms;     // Timeout duration in ms
  
  // Motor test state
  struct {
    int8_t left_pwm;                // Left motor PWM (-100 to +100)
    int8_t right_pwm;               // Right motor PWM (-100 to +100)
    bool emergency_stop;            // Emergency stop flag
  } motor;
  
  // Sensor test state
  struct {
    int8_t target_ir_channel;       // IR channel to test (-1 = all, 0-3 = specific)
    int8_t target_tof_index;        // ToF sensor to test (-1 = all, 0-2 = specific)
    bool stream_raw_data;           // Stream raw sensor values (high frequency)
  } sensor;
  
  // Calibration state
  struct {
    bool active;                    // Calibration in progress
    uint16_t sample_count;          // Number of samples collected
    float ir_min[4];                // IR min values (for white/edge detection)
    float ir_max[4];                // IR max values (for black/ring detection)
    uint16_t tof_min[3];            // ToF min distance (mm)
    uint16_t tof_max[3];            // ToF max distance (mm)
  } calibration;
  
  // Constructor with defaults
  TestModeState() 
    : mode(TestMode::AUTO),
      last_command_ms(0),
      mode_enter_ms(0),
      safety_timeout_enabled(true),
      timeout_duration_ms(5000)  // 5 second default timeout
  {
    motor.left_pwm = 0;
    motor.right_pwm = 0;
    motor.emergency_stop = false;
    
    sensor.target_ir_channel = -1;
    sensor.target_tof_index = -1;
    sensor.stream_raw_data = false;
    
    calibration.active = false;
    calibration.sample_count = 0;
    for (int i = 0; i < 4; i++) {
      calibration.ir_min[i] = 9999.0f;
      calibration.ir_max[i] = 0.0f;
    }
    for (int i = 0; i < 3; i++) {
      calibration.tof_min[i] = 65535;
      calibration.tof_max[i] = 0;
    }
  }
};

// ========== Test Mode Utilities ==========

namespace TestModeUtils {
  
  /**
   * Get test mode as string
   */
  inline const char* getModeString(TestMode mode) {
    switch (mode) {
      case TestMode::AUTO: return "AUTO";
      case TestMode::TEST_MOTOR: return "TEST_MOTOR";
      case TestMode::TEST_SENSOR: return "TEST_SENSOR";
      case TestMode::CALIBRATE_IR: return "CALIBRATE_IR";
      case TestMode::CALIBRATE_TOF: return "CALIBRATE_TOF";
      default: return "UNKNOWN";
    }
  }
  
  /**
   * Check if safety timeout has expired
   */
  inline bool isTimeoutExpired(const TestModeState& state) {
    if (!state.safety_timeout_enabled) return false;
    if (state.mode == TestMode::AUTO) return false;
    
    unsigned long elapsed = millis() - state.last_command_ms;
    return (elapsed > state.timeout_duration_ms);
  }
  
  /**
   * Reset motor state to safe defaults
   */
  inline void safetyStopMotors(TestModeState& state) {
    state.motor.left_pwm = 0;
    state.motor.right_pwm = 0;
    state.motor.emergency_stop = true;
  }
  
  /**
   * Parse test mode from string
   */
  inline TestMode parseModeString(const String& mode_str) {
    if (mode_str == "AUTO") return TestMode::AUTO;
    if (mode_str == "TEST_MOTOR") return TestMode::TEST_MOTOR;
    if (mode_str == "TEST_SENSOR") return TestMode::TEST_SENSOR;
    if (mode_str == "CALIBRATE_IR") return TestMode::CALIBRATE_IR;
    if (mode_str == "CALIBRATE_TOF") return TestMode::CALIBRATE_TOF;
    return TestMode::AUTO;  // Default to AUTO on invalid input
  }
  
  /**
   * Clamp PWM value to valid range
   */
  inline int8_t clampPWM(int value) {
    if (value < -100) return -100;
    if (value > 100) return 100;
    return (int8_t)value;
  }
}

#endif // TEST_MODE_COMMON_H
