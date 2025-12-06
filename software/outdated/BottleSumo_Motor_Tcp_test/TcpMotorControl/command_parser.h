#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>
#include <ArduinoJson.h>

// Command parser for JSONL protocol over TCP
// Handles: set, stop, estop, status, calibrate

// Error codes per contracts/jsonl-commands.md
enum ErrorCode {
  ERR_NONE = 0,
  ERR_BAD_REQUEST,
  ERR_OUT_OF_RANGE,
  ERR_UNSAFE_STATE,
  ERR_INTERNAL_ERROR
};

// Command types
enum CommandAction {
  CMD_NONE = 0,
  CMD_SET,
  CMD_STOP,
  CMD_ESTOP,
  CMD_STATUS,
  CMD_CALIBRATE
};

struct ParsedCommand {
  CommandAction action;
  ErrorCode error;
  
  // Parameters for each command type
  int16_t motorLeft;        // set
  int16_t motorRight;       // set
  String calibrateMode;     // calibrate
  uint16_t calibrateSamples; // calibrate
};

class CommandParser {
public:
  CommandParser();
  
  // Parse a JSONL command string
  // Returns parsed command with action and error code
  ParsedCommand parse(const String& jsonLine);
  
  // Generate response strings
  static String responseOk();
  static String responseError(ErrorCode code, const String& message);
  static String responseStatus(bool auth, int16_t left, int16_t right,
                               const uint16_t sensorRaw[4], const bool sensorFlags[4],
                               bool estop, bool latched);
  
  // Get error code string for logging
  static String getErrorString(ErrorCode code);

private:
  // Parse specific command types
  ErrorCode parseSetCommand(JsonDocument& doc, ParsedCommand& cmd);
  ErrorCode parseStopCommand(JsonDocument& doc, ParsedCommand& cmd);
  ErrorCode parseEstopCommand(JsonDocument& doc, ParsedCommand& cmd);
  ErrorCode parseStatusCommand(JsonDocument& doc, ParsedCommand& cmd);
  ErrorCode parseCalibrateCommand(JsonDocument& doc, ParsedCommand& cmd);
};

#endif // COMMAND_PARSER_H
