#include <Arduino.h>
#include <WiFi.h>

#include "motor_controller.h"
#include "command_parser.h"

// Pin mapping (RP2040 Pico W)
// Left:  LPWM GP11, LDIR GP10
// Right: RPWM GP14, RDIR GP13
static const uint8_t LPWM_PIN = 11;
static const uint8_t LDIR_PIN = 10;
static const uint8_t RPWM_PIN = 14;
static const uint8_t RDIR_PIN = 13;

// WiFi AP credentials
static const char *AP_SSID = "BottleSumo-TCP";
static const char *AP_PASS = "sumo123456"; // 10+ chars recommended

static const uint16_t TCP_PORT = 3333;
static const unsigned long SAFE_OFF_TIMEOUT_MS = 1000; // stop motors if no command within this window

WiFiServer server(TCP_PORT);
WiFiClient client;

MotorController motors(LPWM_PIN, LDIR_PIN, RPWM_PIN, RDIR_PIN);
CommandParser parser;

unsigned long lastCommandMs = 0;
bool authed = false;
String sessionToken = ""; // if non-empty, parser will require matching token

static void startAP() {
#if defined(ARDUINO_ARCH_RP2040)
  // Arduino-Pico core (Pico W)
  WiFi.beginAP(AP_SSID, AP_PASS);
#else
  // Fallback (ESP-like API)
  WiFi.softAP(AP_SSID, AP_PASS);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("[TcpMotorControl] Booting...");

  // Initialize motors (10kHz default)
  motors.begin(10000);
  motors.stopAll();

  // Start WiFi AP + TCP server
  startAP();
  IPAddress ip = WiFi.localIP();
  server.begin();
  Serial.print("AP SSID: "); Serial.println(AP_SSID);
  Serial.print("AP IP: "); Serial.println(ip);
  Serial.print("Listening on port "); Serial.println(TCP_PORT);

  lastCommandMs = millis();
}

// Simple line buffer for JSONL commands
static const size_t LINE_MAX = 512;
char lineBuf[LINE_MAX];
size_t lineLen = 0;

static void resetLineBuf() {
  lineLen = 0;
  lineBuf[0] = '\0';
}

static bool readLineNonBlocking(WiFiClient &c, String &out) {
  while (c.connected() && c.available()) {
    int ch = c.read();
    if (ch < 0) break;
    if (ch == '\r') continue; // ignore CR
    if (ch == '\n') {
      lineBuf[lineLen] = '\0';
      out = String(lineBuf);
      resetLineBuf();
      return true;
    }
    if (lineLen + 1 < LINE_MAX) {
      lineBuf[lineLen++] = (char)ch;
      lineBuf[lineLen] = '\0';
    } else {
      // overflow: reset buffer and report error as a line
      out = String("{\"error\":\"line_too_long\"}");
      resetLineBuf();
      return true;
    }
  }
  return false;
}

static void handleSafeOff() {
  if (millis() - lastCommandMs > SAFE_OFF_TIMEOUT_MS) {
    motors.stopAll();
  }
}

void loop() {
  // Accept new client
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("Client connected");
      client.print("{\"hello\":\"BottleSumo TcpMotorControl\",\"port\":");
      client.print(TCP_PORT);
      client.println("}");
      resetLineBuf();
    }
  }

  // Process client input
  if (client && client.connected()) {
    String line;
    if (readLineNonBlocking(client, line)) {
      if (line.length() == 0) {
        // ignore empty
      } else {
        String resp = parser.handle(line, motors, authed, sessionToken, lastCommandMs);
        if (resp.length()) {
          client.println(resp);
        }
      }
    }
  }

  handleSafeOff();
}
