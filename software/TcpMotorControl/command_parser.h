#pragma once
#include <Arduino.h>
#include "motor_controller.h"

class CommandParser {
public:
  String handle(const String &line,
                MotorController &motors,
                bool &authed,
                String &sessionToken,
                unsigned long &lastCommandMs);

private:
  static String trimQuotes(const String &s);
  static bool hasKey(const String &src, const char *key);
  static String getStr(const String &src, const char *key);
  static bool getNum(const String &src, const char *key, float &out);
};
