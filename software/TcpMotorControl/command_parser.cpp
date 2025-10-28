#include "command_parser.h"

static String jsonOk(const String &msg) {
  String s = "{\"ok\":true,\"msg\":\"" + msg + "\"}";
  return s;
}
static String jsonErr(const String &code, const String &msg) {
  String s = "{\"ok\":false,\"error\":\"" + code + "\",\"msg\":\"" + msg + "\"}";
  return s;
}

String CommandParser::handle(const String &line,
                             MotorController &motors,
                             bool &authed,
                             String &sessionToken,
                             unsigned long &lastCommandMs) {
  String cmd = getStr(line, "cmd");
  if (cmd.length() == 0) {
    // Support legacy key "type"
    cmd = getStr(line, "type");
  }

  // Default to noop if unknown
  if (cmd.equalsIgnoreCase("ping")) {
    return String("{\"ok\":true,\"pong\":") + String(millis()) + "}";
  }

  if (cmd.equalsIgnoreCase("auth")) {
    String tok = getStr(line, "token");
    if (tok.length() < 1) {
      return jsonErr("bad_request", "missing token");
    }
    if (sessionToken.length() == 0) {
      // First-time set requires >=12 chars for safety
      if (tok.length() < 12) {
        return jsonErr("weak_token", "token must be >=12 chars");
      }
      sessionToken = tok;
      authed = true;
      return jsonOk("session established");
    } else {
      if (tok == sessionToken) {
        authed = true;
        return jsonOk("authenticated");
      } else {
        authed = false;
        return jsonErr("unauthorized", "invalid token");
      }
    }
  }

  // Gate commands if a token is set but we're not authenticated
  if (sessionToken.length() > 0 && !authed) {
    return jsonErr("unauthorized", "auth required");
  }

  if (cmd.equalsIgnoreCase("stop")) {
    motors.stopAll();
    lastCommandMs = millis();
    return jsonOk("stopped");
  }

  if (cmd.equalsIgnoreCase("estop")) {
    motors.stopAll();
    authed = false; // require re-auth
    return jsonOk("estop");
  }

  if (cmd.equalsIgnoreCase("drive")) {
    float l = 0, r = 0;
    bool hl = getNum(line, "left", l);
    bool hr = getNum(line, "right", r);
    if (!hl || !hr) {
      return jsonErr("bad_request", "missing left/right");
    }
    // Clamp and drive
    if (l > 100) l = 100; if (l < -100) l = -100;
    if (r > 100) r = 100; if (r < -100) r = -100;

    motors.drive(l, r);
    lastCommandMs = millis();

    String resp = "{\"ok\":true,\"status\":{\"left\":" + String(l) + ",\"right\":" + String(r) + "}}";
    return resp;
  }

  if (cmd.equalsIgnoreCase("status")) {
    String resp = "{\"ok\":true,\"millis\":" + String(millis()) + 
                  ",\"authed\":" + String(authed ? "true" : "false") + 
                  ",\"token_set\":" + String(sessionToken.length() > 0 ? "true" : "false") + "}";
    return resp;
  }

  return jsonErr("unknown_cmd", String("cmd= ") + cmd);
}

String CommandParser::trimQuotes(const String &s) {
  if (s.length() >= 2 && s.charAt(0) == '"' && s.charAt(s.length()-1) == '"') {
    return s.substring(1, s.length()-1);
  }
  return s;
}

bool CommandParser::hasKey(const String &src, const char *key) {
  String pat = String("\"") + key + "\":"; // "key":
  return src.indexOf(pat) >= 0;
}

String CommandParser::getStr(const String &src, const char *key) {
  String pat = String("\"") + key + "\":"; // "key":
  int i = src.indexOf(pat);
  if (i < 0) return String("");
  i += pat.length();
  // Skip spaces
  while (i < (int)src.length() && isspace((unsigned char)src[i])) i++;
  if (i >= (int)src.length()) return String("");
  if (src[i] == '"') {
    int j = src.indexOf('"', i + 1);
    if (j < 0) return String("");
    return src.substring(i + 1, j);
  }
  // Not quoted -> read until comma/brace
  int j = i;
  while (j < (int)src.length() && src[j] != ',' && src[j] != '}' && src[j] != '\n' && src[j] != '\r') j++;
  return src.substring(i, j);
}

bool CommandParser::getNum(const String &src, const char *key, float &out) {
  String s = getStr(src, key);
  if (s.length() == 0) return false;
  out = s.toFloat();
  return true;
}
