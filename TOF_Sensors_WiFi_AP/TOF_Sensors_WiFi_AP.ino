/**************************************************************************
 Pico W BottleSumo Robot - 5 ToF Sensors & WiFi AP Web Dashboard
 - Runs 5 VL53L0X sensors, exposes live readings via web API
 - Creates AP: BottleSumo_Robot / sumo2025
 - Web Dashboard: http://192.168.4.1
 Hardware:
   - Raspberry Pi Pico W
   - VL53L0X x5 via Wire1 (GP26 SDA, GP27 SCL), XSHUT pins: GP11-15
**************************************************************************/

#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <WiFi.h>
#include <WebServer.h>

// --- Hardware Pin Definitions ---
#define XSHUT_1 11 // RIGHT 45°
#define XSHUT_2 12 // RIGHT 22.5°
#define XSHUT_3 13 // FRONT 0°
#define XSHUT_4 14 // LEFT 22.5°
#define XSHUT_5 15 // LEFT 45°

#define NUM_SENSORS 5
const uint8_t xshut_pins[NUM_SENSORS] = {XSHUT_1, XSHUT_2, XSHUT_3, XSHUT_4, XSHUT_5};
const uint8_t sensor_addresses[NUM_SENSORS] = {0x30, 0x31, 0x32, 0x33, 0x34};

// --- Sensor Configuration ---
#define TIMING_BUDGET_US 20000
#define VCSEL_PRE_RANGE 18
#define VCSEL_FINAL_RANGE 14
#define DETECTION_THRESHOLD_MM 1600
#define MAX_VALID_RANGE_MM 2000
#define MIN_VALID_RANGE_MM 30
#define LOOP_DELAY_MS 30
#define MAX_INIT_RETRIES 3

// --- WiFi AP Configuration ---
const char* ap_ssid = "BottleSumo_Robot";
const char* ap_password = "sumo2025";
WebServer server(80);

// --- Sensor Objects & Data ---
Adafruit_VL53L0X sensors[NUM_SENSORS];

struct ToF_Reading {
  uint16_t dist[NUM_SENSORS];
  bool valid[NUM_SENSORS];
  uint8_t status[NUM_SENSORS];
};

// --- Direction labels for dashboard ---
const char* directions[NUM_SENSORS] = {
  "RIGHT", "RIGHT_22", "FRONT", "LEFT_22", "LEFT"
};

// --- Display labels (spaces instead of underscores) ---
const char* display_labels[NUM_SENSORS] = {
  "RIGHT", "RIGHT 22", "FRONT", "LEFT 22", "LEFT"
};

// --- Initialization for 5 sensors ---
bool init_tof_sensors() {
  for (int attempt = 1; attempt <= MAX_INIT_RETRIES; attempt++) {
    Serial.print("ToF init attempt "); Serial.print(attempt); Serial.print("/"); Serial.println(MAX_INIT_RETRIES);
    // Reset all sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
      pinMode(xshut_pins[i], OUTPUT);
      digitalWrite(xshut_pins[i], LOW);
    }
    delay(50);
    bool all_ok = true;
    // Power up each sensor one by one
    for (int i = 0; i < NUM_SENSORS; i++) {
      digitalWrite(xshut_pins[i], HIGH);
      delay(20);
      if (!sensors[i].begin(sensor_addresses[i], false, &Wire1)) {
        Serial.print("Failed to init sensor "); Serial.println(directions[i]);
        all_ok = false;
        break;
      }
      sensors[i].setMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
      sensors[i].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, VCSEL_PRE_RANGE);
      sensors[i].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, VCSEL_FINAL_RANGE);
      Serial.print(directions[i]); Serial.println(": ULTRA FAST mode");
    }
    if (all_ok) {
      Serial.println("All 5 ToF sensors initialized!");
      return true;
    }
    delay(100);
  }
  Serial.println("CRITICAL: Sensor init failed!");
  return false;
}

ToF_Reading read_tof_sensors() {
  ToF_Reading reading;
  VL53L0X_RangingMeasurementData_t data;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensors[i].rangingTest(&data, false);
    reading.status[i] = data.RangeStatus;
    reading.valid[i] = (data.RangeStatus <= 2 &&
                        data.RangeMilliMeter >= MIN_VALID_RANGE_MM &&
                        data.RangeMilliMeter < MAX_VALID_RANGE_MM);
    reading.dist[i] = reading.valid[i] ? data.RangeMilliMeter : 0;
  }
  return reading;
}

// --- Object direction logic (returns "CLEAR" or list of directions) ---
String get_object_direction(ToF_Reading &reading, uint16_t threshold = DETECTION_THRESHOLD_MM) {
  String dirs = "";
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (reading.valid[i] && reading.dist[i] < threshold) {
      dirs += directions[i]; dirs += " ";
    }
  }
  return dirs.length() ? dirs : "CLEAR";
}

// --- JSON output for web API ---
String get_json_reading(ToF_Reading &reading) {
  String json = "{";
  for (int i = 0; i < NUM_SENSORS; i++) {
    json += "\"" + String(directions[i]) + "\":{";
    json += "\"distance\":" + String(reading.dist[i]) + ",";
    json += "\"valid\":" + String(reading.valid[i] ? "true" : "false") + ",";
    json += "\"status\":" + String(reading.status[i]);
    json += "},";
  }
  json += "\"direction\":\"" + get_object_direction(reading) + "\",";
  json += "\"timestamp\":" + String(millis());
  json += "}";
  return json;
}

// --- Compact Serial print ---
void print_tof_readings(ToF_Reading &reading) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(directions[i]); Serial.print(":");
    if (reading.valid[i]) Serial.print(reading.dist[i]);
    else { Serial.print("ERR("); Serial.print(reading.status[i]); Serial.print(")"); }
    Serial.print(" ");
  }
  Serial.print("-> "); Serial.println(get_object_direction(reading));
}

// --- I2C scan for debugging ---
void scanI2C() {
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x"); Serial.println(address, HEX); nDevices++;
    }
  }
  if (!nDevices) Serial.println("No I2C devices found");
  else Serial.println("done");
}

// --- WiFi Access Point Setup ---
void setupWiFiAP() {
  Serial.println("=== Setting up WiFi AP ===");
  WiFi.mode(WIFI_AP);
  bool ap_ok = WiFi.softAP(ap_ssid, ap_password);
  if (ap_ok) {
    IPAddress IP = WiFi.softAPIP();
    Serial.println("AP OK! SSID: " + String(ap_ssid) + " IP: " + IP.toString());
  } else {
    Serial.println("AP FAILED!");
  }
}

// --- Web Server Logic ---
void setupWebServer() {
  // Root dashboard
  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
    html += "<title>BottleSumo Robot - 5 ToF Sensors</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>body{font-family:Arial;margin:20px;background:#f0f0f0;} .container{max-width:900px;margin:0 auto;background:#fff;padding:20px;border-radius:10px;box-shadow:0 2px 8px #aaa;} .sensor{display:inline-block;width:18%;margin:1%;padding:12px;background:#e8f4f8;border-radius:5px;text-align:center;} .sensor h3{margin:0 0 6px;color:#0066cc;} .distance{font-size:20px;font-weight:bold;color:#00aa00;} .invalid{color:#cc0000;} .direction{font-size:28px;font-weight:bold;margin:18px;padding:18px;background:#ffe;border-radius:10px;text-align:center;} .api-info{margin-top:24px;padding:12px;background:#f5f5f5;border-radius:5px;} code{background:#ddd;padding:2px 6px;border-radius:3px;} </style>";
    html += "<script>function updateData(){fetch('/data').then(r=>r.json()).then(d=>{";
    for (int i = 0; i < NUM_SENSORS; i++) {
      html += "document.getElementById('" + String(directions[i]) + "-dist').textContent = d['" + String(directions[i]) + "'].valid ? d['" + String(directions[i]) + "'].distance+'mm' : 'ERROR ('+d['" + String(directions[i]) + "'].status+')';";
      html += "document.getElementById('" + String(directions[i]) + "-dist').className = d['" + String(directions[i]) + "'].valid ? 'distance' : 'distance invalid';";
    }
    html += "document.getElementById('direction').textContent = d.direction;";
    html += "});} setInterval(updateData,100); updateData();</script>";
    html += "</head><body><div class='container'><h1>🤖 Bottle Sumo Robot</h1><h2>5 ToF Sensor Monitor</h2><div style='text-align:center;'>";
    for (int i = 0; i < NUM_SENSORS; i++) {
      html += "<div class='sensor'><h3>" + String(display_labels[i]) + "</h3><div id='" + String(directions[i]) + "-dist' class='distance'>---</div></div>";
    }
    html += "</div><div class='direction'><div style='font-size:14px;color:#666;'>DETECTION</div><div id='direction'>CLEAR</div></div>";
    html += "<div class='api-info'><h3>API Endpoints:</h3><p><code>GET /data</code> - JSON sensor readings</p><p><code>GET /status</code> - WiFi/system info</p><p>Update rate: ~10Hz (100ms)</p></div></div></body></html>";
    server.send(200, "text/html", html);
  });

  // JSON data endpoint
  server.on("/data", HTTP_GET, []() {
    static ToF_Reading latest;
    latest = read_tof_sensors();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", get_json_reading(latest));
  });

  // Status endpoint
  server.on("/status", HTTP_GET, []() {
    String json = "{";
    json += "\"wifi_ssid\":\"" + String(ap_ssid) + "\",";
    json += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
    json += "\"connected_clients\":" + String(WiFi.softAPgetStationNum()) + ",";
    json += "\"uptime_ms\":" + String(millis());
    json += "}";
    server.send(200, "application/json", json);
  });

  server.onNotFound([]() { server.send(404, "text/plain", "404 Not Found"); });
  server.begin();
  Serial.println("HTTP server started! Endpoints: / /data /status");
}

// --- Setup & Loop ---
void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000));
  Wire1.setSDA(26); Wire1.setSCL(27); Wire1.begin();
  Serial.println("=== BottleSumo Robot - 5x ToF Sensors WiFi AP ===");
  if (!init_tof_sensors()) { Serial.println("Sensor init failed!"); while(1); }
  scanI2C();
  setupWiFiAP();
  setupWebServer();
  Serial.println("System ready. Web dashboard at http://192.168.4.1");
}

void loop() {
  server.handleClient();
  static ToF_Reading latest;
  latest = read_tof_sensors();
  print_tof_readings(latest);
  String dir = get_object_direction(latest, DETECTION_THRESHOLD_MM);
  if (dir != "CLEAR") { Serial.print("DETECTED: "); Serial.println(dir); }
  delay(LOOP_DELAY_MS);
}