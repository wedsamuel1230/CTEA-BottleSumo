#include "command_parser.h"

CommandParser::CommandParser() {
}

ParsedCommand CommandParser::parse(const String& jsonLine) {
  ParsedCommand cmd;
  cmd.action = CMD_NONE;
  cmd.error = ERR_NONE;
  cmd.motorLeft = 0;
  cmd.motorRight = 0;
  cmd.calibrateSamples = 64; // default
  
  // Parse JSON
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, jsonLine);
  
  if (error) {
    cmd.error = ERR_BAD_REQUEST;
    return cmd;
  }
  
  // Extract action field
  if (!doc["action"].is<String>()) {
    cmd.error = ERR_BAD_REQUEST;
    return cmd;
  }
  
  String action = doc["action"].as<String>();
  
  // Route to specific parser (no auth required)
  if (action == "set") {
    cmd.action = CMD_SET;
    cmd.error = parseSetCommand(doc, cmd);
  }
  else if (action == "stop") {
    cmd.action = CMD_STOP;
    cmd.error = parseStopCommand(doc, cmd);
  }
  else if (action == "estop") {
    cmd.action = CMD_ESTOP;
    cmd.error = parseEstopCommand(doc, cmd);
  }
  else if (action == "status") {
    cmd.action = CMD_STATUS;
    cmd.error = parseStatusCommand(doc, cmd);
  }
  else if (action == "calibrate") {
    cmd.action = CMD_CALIBRATE;
    cmd.error = parseCalibrateCommand(doc, cmd);
  }
  else {
    cmd.error = ERR_BAD_REQUEST;
  }
  
  return cmd;
}

ErrorCode CommandParser::parseSetCommand(JsonDocument& doc, ParsedCommand& cmd) {
  if (!doc["left"].is<int>() || !doc["right"].is<int>()) {
    return ERR_BAD_REQUEST;
  }
  
  // Extract and validate range
  int left = doc["left"].as<int>();
  int right = doc["right"].as<int>();
  
  if (left < -255 || left > 255 || right < -255 || right > 255) {
    return ERR_OUT_OF_RANGE;
  }
  
  cmd.motorLeft = (int16_t)left;
  cmd.motorRight = (int16_t)right;
  return ERR_NONE;
}

ErrorCode CommandParser::parseStopCommand(JsonDocument& doc, ParsedCommand& cmd) {
  // No additional parameters
  return ERR_NONE;
}

ErrorCode CommandParser::parseEstopCommand(JsonDocument& doc, ParsedCommand& cmd) {
  // No additional parameters
  return ERR_NONE;
}

ErrorCode CommandParser::parseStatusCommand(JsonDocument& doc, ParsedCommand& cmd) {
  // No additional parameters
  return ERR_NONE;
}

ErrorCode CommandParser::parseCalibrateCommand(JsonDocument& doc, ParsedCommand& cmd) {
  if (!doc["mode"].is<String>()) {
    return ERR_BAD_REQUEST;
  }
  
  cmd.calibrateMode = doc["mode"].as<String>();
  
  if (doc["samples"].is<uint16_t>()) {
    cmd.calibrateSamples = doc["samples"].as<uint16_t>();
  }
  
  return ERR_NONE;
}

String CommandParser::responseOk() {
  JsonDocument doc;
  doc["status"] = "ok";
  String response;
  serializeJson(doc, response);
  return response;
}

String CommandParser::responseError(ErrorCode code, const String& message) {
  JsonDocument doc;
  doc["status"] = "error";
  doc["code"] = getErrorString(code);
  doc["message"] = message;
  String response;
  serializeJson(doc, response);
  return response;
}

String CommandParser::responseStatus(bool auth, int16_t left, int16_t right,
                                     const uint16_t sensorRaw[4], const bool sensorFlags[4],
                                     bool estop, bool latched) {
  JsonDocument doc;
  doc["status"] = "ok";
  doc["auth"] = auth;
  
  JsonObject motors = doc["motors"].to<JsonObject>();
  motors["left"] = left;
  motors["right"] = right;
  
  JsonObject sensors = doc["sensors"].to<JsonObject>();
  JsonArray raw = sensors["raw"].to<JsonArray>();
  JsonArray flags = sensors["flags"].to<JsonArray>();
  for (int i = 0; i < 4; i++) {
    raw.add(sensorRaw[i]);
    flags.add(sensorFlags[i]);
  }
  
  JsonObject safety = doc["safety"].to<JsonObject>();
  safety["estop"] = estop;
  safety["latched"] = latched;
  
  String response;
  serializeJson(doc, response);
  return response;
}

String CommandParser::getErrorString(ErrorCode code) {
  switch (code) {
    case ERR_BAD_REQUEST: return "BAD_REQUEST";
    case ERR_OUT_OF_RANGE: return "OUT_OF_RANGE";
    case ERR_UNSAFE_STATE: return "UNSAFE_STATE";
    case ERR_INTERNAL_ERROR: return "INTERNAL_ERROR";
    default: return "UNKNOWN";
  }
}
