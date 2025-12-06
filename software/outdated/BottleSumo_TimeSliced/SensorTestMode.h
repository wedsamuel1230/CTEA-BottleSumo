/**
 * SensorTestMode.h - Sensor Testing and Calibration Module
 * 
 * Modular sensor test mode for IR and ToF sensor testing and calibration.
 * Designed for easy extraction and reuse in TimeSliced version.
 * 
 * Features:
 * - Individual sensor channel testing
 * - Runtime threshold adjustment
 * - Calibration mode (min/max recording)
 * - High-frequency raw data streaming
 * 
 * TCP Commands:
 * - TEST_SENSOR IR <channel>       Test IR sensor (0-3 or -1 for all)
 * - TEST_SENSOR TOF <index>        Test ToF sensor (0-2 or -1 for all)
 * - CALIBRATE_IR START             Start IR calibration
 * - CALIBRATE_IR STOP              Stop and save IR calibration
 * - CALIBRATE_TOF START            Start ToF calibration
 * - CALIBRATE_TOF STOP             Stop and save ToF calibration
 * - GET_CALIBRATION                Get calibration results
 * 
 * Author: CTEA-BottleSumo Project
 * Date: 2025-10-15
 * License: MIT
 */

#ifndef SENSOR_TEST_MODE_H
#define SENSOR_TEST_MODE_H

#include <Arduino.h>
#include "TestModeCommon.h"

namespace SensorTestMode {
  
  /**
   * Handle TEST_SENSOR command
   * Format: TEST_SENSOR <type> <index>
   * Type: IR or TOF
   * Index: -1 for all, or specific channel/sensor number
   * 
   * @param args Command arguments string
   * @param state Test mode state to update
   * @return Response string
   */
  inline String handleTestSensorCommand(const String& args, TestModeState& state) {
    // Parse arguments
    int space_pos = args.indexOf(' ');
    if (space_pos == -1) {
      return "ERROR: TEST_SENSOR requires format: <IR|TOF> <index>\n";
    }
    
    String sensor_type = args.substring(0, space_pos);
    sensor_type.toUpperCase();
    int sensor_index = args.substring(space_pos + 1).toInt();
    
    if (sensor_type == "IR") {
      if (sensor_index < -1 || sensor_index > 3) {
        return "ERROR: IR channel must be -1 (all) or 0-3\n";
      }
      state.sensor.target_ir_channel = sensor_index;
      state.sensor.target_tof_index = -2;  // Disable ToF testing
      state.sensor.stream_raw_data = true;
      state.last_command_ms = millis();
      return String("OK: Testing IR channel ") + (sensor_index == -1 ? "ALL" : String(sensor_index)) + "\n";
      
    } else if (sensor_type == "TOF") {
      if (sensor_index < -1 || sensor_index > 2) {
        return "ERROR: ToF index must be -1 (all) or 0-2\n";
      }
      state.sensor.target_tof_index = sensor_index;
      state.sensor.target_ir_channel = -2;  // Disable IR testing
      state.sensor.stream_raw_data = true;
      state.last_command_ms = millis();
      return String("OK: Testing ToF sensor ") + (sensor_index == -1 ? "ALL" : String(sensor_index)) + "\n";
      
    } else {
      return "ERROR: Sensor type must be IR or TOF\n";
    }
  }
  
  /**
   * Handle CALIBRATE_IR command
   * Format: CALIBRATE_IR <START|STOP>
   * 
   * @param args Command arguments (START or STOP)
   * @param state Test mode state to update
   * @return Response string
   */
  inline String handleCalibrateIRCommand(const String& args, TestModeState& state) {
    String action = args;
    action.toUpperCase();
    action.trim();
    
    if (action == "START") {
      state.calibration.active = true;
      state.calibration.sample_count = 0;
      // Reset min/max
      for (int i = 0; i < 4; i++) {
        state.calibration.ir_min[i] = 9999.0f;
        state.calibration.ir_max[i] = 0.0f;
      }
      state.last_command_ms = millis();
      return "OK: IR calibration started. Move sensor over white (edge) and black (ring) surfaces.\n";
      
    } else if (action == "STOP") {
      state.calibration.active = false;
      String result = "OK: IR calibration stopped. Samples: " + String(state.calibration.sample_count) + "\n";
      result += "Results:\n";
      for (int i = 0; i < 4; i++) {
        result += "  CH" + String(i) + ": MIN=" + String(state.calibration.ir_min[i], 3) + 
                  "V MAX=" + String(state.calibration.ir_max[i], 3) + "V\n";
        // Suggested threshold = midpoint
        float suggested = (state.calibration.ir_min[i] + state.calibration.ir_max[i]) / 2.0f;
        result += "       Suggested threshold: " + String(suggested, 3) + "V\n";
      }
      return result;
      
    } else {
      return "ERROR: CALIBRATE_IR requires START or STOP\n";
    }
  }
  
  /**
   * Handle CALIBRATE_TOF command
   * Format: CALIBRATE_TOF <START|STOP>
   * 
   * @param args Command arguments (START or STOP)
   * @param state Test mode state to update
   * @return Response string
   */
  inline String handleCalibrateToFCommand(const String& args, TestModeState& state) {
    String action = args;
    action.toUpperCase();
    action.trim();
    
    if (action == "START") {
      state.calibration.active = true;
      state.calibration.sample_count = 0;
      // Reset min/max
      for (int i = 0; i < 3; i++) {
        state.calibration.tof_min[i] = 65535;
        state.calibration.tof_max[i] = 0;
      }
      state.last_command_ms = millis();
      return "OK: ToF calibration started. Move objects at varying distances.\n";
      
    } else if (action == "STOP") {
      state.calibration.active = false;
      String result = "OK: ToF calibration stopped. Samples: " + String(state.calibration.sample_count) + "\n";
      result += "Results:\n";
      for (int i = 0; i < 3; i++) {
        result += "  ToF" + String(i) + ": MIN=" + String(state.calibration.tof_min[i]) + 
                  "mm MAX=" + String(state.calibration.tof_max[i]) + "mm\n";
      }
      return result;
      
    } else {
      return "ERROR: CALIBRATE_TOF requires START or STOP\n";
    }
  }
  
  /**
   * Handle GET_CALIBRATION command
   * 
   * @param state Test mode state
   * @return Response string with calibration data
   */
  inline String handleGetCalibrationCommand(const TestModeState& state) {
    String response = "Calibration Status:\n";
    response += "  Active: " + String(state.calibration.active ? "YES" : "NO") + "\n";
    response += "  Samples: " + String(state.calibration.sample_count) + "\n\n";
    
    response += "IR Sensors:\n";
    for (int i = 0; i < 4; i++) {
      response += "  CH" + String(i) + ": MIN=" + String(state.calibration.ir_min[i], 3) + 
                  "V MAX=" + String(state.calibration.ir_max[i], 3) + "V";
      if (state.calibration.ir_max[i] > 0) {
        float suggested = (state.calibration.ir_min[i] + state.calibration.ir_max[i]) / 2.0f;
        response += " (Threshold: " + String(suggested, 3) + "V)";
      }
      response += "\n";
    }
    
    response += "\nToF Sensors:\n";
    for (int i = 0; i < 3; i++) {
      response += "  ToF" + String(i) + ": MIN=" + String(state.calibration.tof_min[i]) + 
                  "mm MAX=" + String(state.calibration.tof_max[i]) + "mm\n";
    }
    
    return response;
  }
  
  /**
   * Update calibration data with current sensor readings
   * Call this in loop() when calibration.active == true
   * 
   * @param state Test mode state to update
   * @param ir_voltages Array of 4 IR sensor voltages
   * @param tof_distances Array of 3 ToF distances (mm)
   * @param tof_valid Array of 3 ToF validity flags
   */
  inline void updateCalibration(TestModeState& state, 
                                const float* ir_voltages,
                                const uint16_t* tof_distances,
                                const bool* tof_valid) {
    if (!state.calibration.active) return;
    
    state.calibration.sample_count++;
    
    // Update IR min/max
    for (int i = 0; i < 4; i++) {
      if (ir_voltages[i] < state.calibration.ir_min[i]) {
        state.calibration.ir_min[i] = ir_voltages[i];
      }
      if (ir_voltages[i] > state.calibration.ir_max[i]) {
        state.calibration.ir_max[i] = ir_voltages[i];
      }
    }
    
    // Update ToF min/max (only if valid)
    for (int i = 0; i < 3; i++) {
      if (tof_valid[i]) {
        if (tof_distances[i] < state.calibration.tof_min[i]) {
          state.calibration.tof_min[i] = tof_distances[i];
        }
        if (tof_distances[i] > state.calibration.tof_max[i]) {
          state.calibration.tof_max[i] = tof_distances[i];
        }
      }
    }
  }
  
  /**
   * Build JSON sensor test data for streaming
   * 
   * @param state Test mode state
   * @param ir_voltages Array of 4 IR sensor voltages
   * @param tof_distances Array of 3 ToF distances (mm)
   * @param tof_valid Array of 3 ToF validity flags
   * @return JSON string fragment (without outer braces)
   */
  inline String buildSensorTestJSON(const TestModeState& state,
                                    const float* ir_voltages,
                                    const uint16_t* tof_distances,
                                    const bool* tof_valid) {
    String json = "\"sensor_test\":{";
    json += "\"target_ir\":" + String(state.sensor.target_ir_channel) + ",";
    json += "\"target_tof\":" + String(state.sensor.target_tof_index) + ",";
    json += "\"ir_raw\":[" + String(ir_voltages[0], 3) + "," + String(ir_voltages[1], 3) + "," +
            String(ir_voltages[2], 3) + "," + String(ir_voltages[3], 3) + "],";
    json += "\"tof_raw\":[" + String(tof_distances[0]) + "," + String(tof_distances[1]) + "," +
            String(tof_distances[2]) + "],";
    json += "\"tof_valid\":[" + String(tof_valid[0] ? "true" : "false") + "," +
            String(tof_valid[1] ? "true" : "false") + "," +
            String(tof_valid[2] ? "true" : "false") + "]";
    json += "}";
    return json;
  }
  
  /**
   * Build JSON calibration status for streaming
   * 
   * @param state Test mode state
   * @return JSON string fragment (without outer braces)
   */
  inline String buildCalibrationJSON(const TestModeState& state) {
    String json = "\"calibration\":{";
    json += "\"active\":" + String(state.calibration.active ? "true" : "false") + ",";
    json += "\"samples\":" + String(state.calibration.sample_count) + ",";
    json += "\"ir_min\":[" + String(state.calibration.ir_min[0], 3) + "," +
            String(state.calibration.ir_min[1], 3) + "," +
            String(state.calibration.ir_min[2], 3) + "," +
            String(state.calibration.ir_min[3], 3) + "],";
    json += "\"ir_max\":[" + String(state.calibration.ir_max[0], 3) + "," +
            String(state.calibration.ir_max[1], 3) + "," +
            String(state.calibration.ir_max[2], 3) + "," +
            String(state.calibration.ir_max[3], 3) + "],";
    json += "\"tof_min\":[" + String(state.calibration.tof_min[0]) + "," +
            String(state.calibration.tof_min[1]) + "," +
            String(state.calibration.tof_min[2]) + "],";
    json += "\"tof_max\":[" + String(state.calibration.tof_max[0]) + "," +
            String(state.calibration.tof_max[1]) + "," +
            String(state.calibration.tof_max[2]) + "]";
    json += "}";
    return json;
  }
}

#endif // SENSOR_TEST_MODE_H
