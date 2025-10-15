/**
 * MotorTestMode.h - Motor Testing Module
 * 
 * Modular motor test mode with GUI control and safety features.
 * Designed for easy extraction and reuse in TimeSliced version.
 * 
 * Features:
 * - Direct PWM control via TCP/GUI
 * - Safety timeout and emergency stop
 * - Individual motor testing
 * - Status monitoring
 * 
 * TCP Commands:
 * - TEST_MOTOR <left> <right>    Set motor PWM (-100 to +100)
 * - STOP_MOTOR                   Emergency stop
 * - GET_MOTOR                    Get current motor status
 * 
 * Author: CTEA-BottleSumo Project
 * Date: 2025-10-15
 * License: MIT
 */

#ifndef MOTOR_TEST_MODE_H
#define MOTOR_TEST_MODE_H

#include <Arduino.h>
#include "TestModeCommon.h"
#include "Motor.h"

namespace MotorTestMode {
  
  /**
   * Handle TEST_MOTOR command
   * Format: TEST_MOTOR <left_pwm> <right_pwm>
   * 
   * @param args Command arguments string
   * @param state Test mode state to update
   * @param motor_left Left motor object
   * @param motor_right Right motor object
   * @return Response string
   */
  inline String handleTestMotorCommand(const String& args, TestModeState& state, 
                                       Motor& motor_left, Motor& motor_right) {
    // Parse arguments
    int space_pos = args.indexOf(' ');
    if (space_pos == -1) {
      return "ERROR: TEST_MOTOR requires 2 arguments: <left> <right>\n";
    }
    
    int left_pwm = args.substring(0, space_pos).toInt();
    int right_pwm = args.substring(space_pos + 1).toInt();
    
    // Clamp values
    left_pwm = TestModeUtils::clampPWM(left_pwm);
    right_pwm = TestModeUtils::clampPWM(right_pwm);
    
    // Update state
    state.motor.left_pwm = left_pwm;
    state.motor.right_pwm = right_pwm;
    state.motor.emergency_stop = false;
    state.last_command_ms = millis();
    
    // Apply PWM
    motor_left.setDuty(left_pwm);
    motor_right.setDuty(right_pwm);
    
    return String("OK: Motors set to LEFT=") + left_pwm + " RIGHT=" + right_pwm + "\n";
  }
  
  /**
   * Handle STOP_MOTOR command
   * 
   * @param state Test mode state to update
   * @param motor_left Left motor object
   * @param motor_right Right motor object
   * @return Response string
   */
  inline String handleStopMotorCommand(TestModeState& state, 
                                       Motor& motor_left, Motor& motor_right) {
    state.motor.left_pwm = 0;
    state.motor.right_pwm = 0;
    state.motor.emergency_stop = true;
    
    motor_left.stop();
    motor_right.stop();
    
    return "OK: Emergency stop activated\n";
  }
  
  /**
   * Handle GET_MOTOR command
   * 
   * @param state Test mode state
   * @return Response string with motor status
   */
  inline String handleGetMotorCommand(const TestModeState& state) {
    String response = "Motor Status:\n";
    response += "  Left PWM:  " + String(state.motor.left_pwm) + "%\n";
    response += "  Right PWM: " + String(state.motor.right_pwm) + "%\n";
    response += "  E-Stop:    " + String(state.motor.emergency_stop ? "ACTIVE" : "INACTIVE") + "\n";
    response += "  Last Cmd:  " + String((millis() - state.last_command_ms) / 1000) + "s ago\n";
    return response;
  }
  
  /**
   * Execute motor test mode logic (call in loop())
   * Handles safety timeout and motor updates
   * 
   * @param state Test mode state
   * @param motor_left Left motor object
   * @param motor_right Right motor object
   */
  inline void executeTestMode(TestModeState& state, Motor& motor_left, Motor& motor_right) {
    // Check safety timeout
    if (TestModeUtils::isTimeoutExpired(state) && !state.motor.emergency_stop) {
      Serial.println("⚠️ Motor test timeout - Emergency stop!");
      TestModeUtils::safetyStopMotors(state);
      motor_left.stop();
      motor_right.stop();
    }
    
    // Apply current PWM values (in case they were updated)
    if (!state.motor.emergency_stop) {
      motor_left.setDuty(state.motor.left_pwm);
      motor_right.setDuty(state.motor.right_pwm);
    }
  }
  
  /**
   * Build JSON motor status for streaming
   * 
   * @param state Test mode state
   * @return JSON string fragment (without outer braces)
   */
  inline String buildMotorJSON(const TestModeState& state) {
    String json = "\"motors\":{";
    json += "\"left\":" + String(state.motor.left_pwm) + ",";
    json += "\"right\":" + String(state.motor.right_pwm) + ",";
    json += "\"estop\":" + String(state.motor.emergency_stop ? "true" : "false") + ",";
    json += "\"timeout_sec\":" + String((millis() - state.last_command_ms) / 1000);
    json += "}";
    return json;
  }
}

#endif // MOTOR_TEST_MODE_H
