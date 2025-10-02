#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <WiFi.h>
#include <WebServer.h>

// Hardware Pin Definitions
#define XSHUT_1 11 // GP11 - Right sensor
#define XSHUT_2 12 // GP12 - Front sensor  
#define XSHUT_3 13 // GP13 - Left sensor

// Sensor Configuration Constants
#define TIMING_BUDGET_US 20000          // 20ms per reading for ultra-fast mode
#define VCSEL_PRE_RANGE 18              // Pre-range pulse period (speed optimized)
#define VCSEL_FINAL_RANGE 14            // Final-range pulse period (speed optimized)
#define DETECTION_THRESHOLD_MM 1600     // 160cm - sumo ring detection distance
#define MAX_VALID_RANGE_MM 2000         // Maximum sensor range
#define MIN_VALID_RANGE_MM 30           // Minimum sensor range (avoid false positives)
#define LOOP_DELAY_MS 30                // 30ms = ~33Hz update rate
#define MAX_INIT_RETRIES 3              // Maximum sensor initialization attempts

// WiFi AP Configuration
const char* ap_ssid = "BottleSumo_Robot";      // WiFi network name
const char* ap_password = "sumo2025";          // Password (min 8 characters)

Adafruit_VL53L0X lox1, lox2, lox3;
WebServer server(80);

// Global variable to store latest reading
ToF_Reading latest_reading;

struct ToF_Reading {
  uint16_t right_distance;
  uint16_t front_distance;
  uint16_t left_distance;
  bool right_valid;
  bool front_valid;
  bool left_valid;
  uint8_t right_status;
  uint8_t front_status;
  uint8_t left_status;
};

// ULTRA FAST initialization with retry mechanism
bool init_tof_sensors() {
  for (int attempt = 1; attempt <= MAX_INIT_RETRIES; attempt++) {
    Serial.print("Initialization attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(MAX_INIT_RETRIES);
    
    // Reset all sensors
    pinMode(XSHUT_1, OUTPUT);
    pinMode(XSHUT_2, OUTPUT);
    pinMode(XSHUT_3, OUTPUT);
    digitalWrite(XSHUT_1, LOW);
    digitalWrite(XSHUT_2, LOW);
    digitalWrite(XSHUT_3, LOW);
    delay(50); // Full reset delay
    
    bool all_sensors_ok = true;

    // Right sensor - ULTRA FAST timing
    digitalWrite(XSHUT_1, HIGH);
    delay(20); // Minimal delay
    if (!lox1.begin(0x30, false, &Wire1)) {
      Serial.println("Failed to init RIGHT sensor");
      all_sensors_ok = false;
      delay(100); // Wait before retry
      continue;
    }
    
    // ULTRA FAST: 20ms timing budget
    lox1.setMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
    lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, VCSEL_PRE_RANGE);
    lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, VCSEL_FINAL_RANGE);
    Serial.println("Right sensor: ULTRA FAST mode (20ms)");

    // Front sensor
    digitalWrite(XSHUT_2, HIGH);
    delay(20);
    if (!lox2.begin(0x31, false, &Wire1)) {
      Serial.println("Failed to init FRONT sensor");
      all_sensors_ok = false;
      delay(100);
      continue;
    }
    
    lox2.setMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
    lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, VCSEL_PRE_RANGE);
    lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, VCSEL_FINAL_RANGE);
    Serial.println("Front sensor: ULTRA FAST mode (20ms)");

    // Left sensor
    digitalWrite(XSHUT_3, HIGH);
    delay(20);
    if (!lox3.begin(0x32, false, &Wire1)) {
      Serial.println("Failed to init LEFT sensor");
      all_sensors_ok = false;
      delay(100);
      continue;
    }
    
    lox3.setMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
    lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, VCSEL_PRE_RANGE);
    lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, VCSEL_FINAL_RANGE);
    Serial.println("Left sensor: ULTRA FAST mode (20ms)");

    if (all_sensors_ok) {
      Serial.println("All sensors initialized successfully!");
      return true;
    }
  }
  
  Serial.println("CRITICAL: Failed to initialize all sensors after multiple retries!");
  return false;
}

ToF_Reading read_tof_sensors() {
  ToF_Reading reading;
  VL53L0X_RangingMeasurementData_t data1, data2, data3;

  lox1.rangingTest(&data1, false);
  lox2.rangingTest(&data2, false);
  lox3.rangingTest(&data3, false);

  reading.right_status = data1.RangeStatus;
  reading.front_status = data2.RangeStatus;
  reading.left_status = data3.RangeStatus;

  // Accept status 0-2 for reliable readings (0=valid, 1=sigma fail, 2=signal fail)
  // Status 4 (phase fail) often means no object - can cause false positives
  reading.right_valid = (data1.RangeStatus <= 2 && 
                         data1.RangeMilliMeter >= MIN_VALID_RANGE_MM && 
                         data1.RangeMilliMeter < MAX_VALID_RANGE_MM);
  reading.right_distance = reading.right_valid ? data1.RangeMilliMeter : 0;

  reading.front_valid = (data2.RangeStatus <= 2 && 
                         data2.RangeMilliMeter >= MIN_VALID_RANGE_MM && 
                         data2.RangeMilliMeter < MAX_VALID_RANGE_MM);
  reading.front_distance = reading.front_valid ? data2.RangeMilliMeter : 0;

  reading.left_valid = (data3.RangeStatus <= 2 && 
                        data3.RangeMilliMeter >= MIN_VALID_RANGE_MM && 
                        data3.RangeMilliMeter < MAX_VALID_RANGE_MM);
  reading.left_distance = reading.left_valid ? data3.RangeMilliMeter : 0;

  return reading;
}

String get_object_direction(ToF_Reading &reading, uint16_t detection_threshold = DETECTION_THRESHOLD_MM) {
  String directions = "";

  if (reading.front_valid && reading.front_distance < detection_threshold) {
    directions += "FRONT ";
  }
  if (reading.right_valid && reading.right_distance < detection_threshold) {
    directions += "RIGHT ";
  }
  if (reading.left_valid && reading.left_distance < detection_threshold) {
    directions += "LEFT ";
  }

  if (directions.length() == 0) {
    return "CLEAR";
  }

  directions.trim();
  return directions;
}

// JSON output for web API
String get_json_reading(ToF_Reading &reading) {
  String json = "{";
  
  // Right sensor
  json += "\"right\":{";
  json += "\"distance\":" + String(reading.right_distance) + ",";
  json += "\"valid\":" + String(reading.right_valid ? "true" : "false") + ",";
  json += "\"status\":" + String(reading.right_status);
  json += "},";
  
  // Front sensor
  json += "\"front\":{";
  json += "\"distance\":" + String(reading.front_distance) + ",";
  json += "\"valid\":" + String(reading.front_valid ? "true" : "false") + ",";
  json += "\"status\":" + String(reading.front_status);
  json += "},";
  
  // Left sensor
  json += "\"left\":{";
  json += "\"distance\":" + String(reading.left_distance) + ",";
  json += "\"valid\":" + String(reading.left_valid ? "true" : "false") + ",";
  json += "\"status\":" + String(reading.left_status);
  json += "},";
  
  // Direction
  json += "\"direction\":\"" + get_object_direction(reading, DETECTION_THRESHOLD_MM) + "\",";
  
  // Timestamp
  json += "\"timestamp\":" + String(millis());
  
  json += "}";
  return json;
}

// Compact printing for speed
void print_tof_readings(ToF_Reading &reading) {
  Serial.print("R:");
  if (reading.right_valid) {
    Serial.print(reading.right_distance);
  } else {
    Serial.print("ERR("); Serial.print(reading.right_status); Serial.print(")");
  }
  
  Serial.print(" F:");
  if (reading.front_valid) {
    Serial.print(reading.front_distance);
  } else {
    Serial.print("ERR("); Serial.print(reading.front_status); Serial.print(")");
  }
  
  Serial.print(" L:");
  if (reading.left_valid) {
    Serial.print(reading.left_distance);
  } else {
    Serial.print("ERR("); Serial.print(reading.left_status); Serial.print(")");
  }
  
  Serial.print(" -> ");
  Serial.println(get_object_direction(reading, DETECTION_THRESHOLD_MM));
}

void scanI2C() {
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) Serial.println("No I2C devices found");
  else Serial.println("done");
}

// ============== WiFi AP Mode Setup ==============

void setupWiFiAP() {
  Serial.println("\n=== Setting up WiFi Access Point ===");
  
  // Configure Access Point
  WiFi.mode(WIFI_AP);
  bool ap_success = WiFi.softAP(ap_ssid, ap_password);
  
  if (ap_success) {
    Serial.println("Access Point created successfully!");
    Serial.print("SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Password: ");
    Serial.println(ap_password);
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    Serial.println("\nConnect your device to this WiFi network");
    Serial.print("Then access: http://");
    Serial.println(IP);
  } else {
    Serial.println("FAILED to create Access Point!");
  }
}

void setupWebServer() {
  // Root page - provides info
  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
    html += "<title>Bottle Sumo Robot - ToF Sensors</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>";
    html += "body{font-family:Arial,sans-serif;margin:20px;background:#f0f0f0;}";
    html += ".container{max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);}";
    html += "h1{color:#333;text-align:center;}";
    html += ".sensor{display:inline-block;width:30%;margin:1%;padding:15px;background:#e8f4f8;border-radius:5px;text-align:center;}";
    html += ".sensor h3{margin:5px 0;color:#0066cc;}";
    html += ".distance{font-size:24px;font-weight:bold;color:#00aa00;}";
    html += ".invalid{color:#cc0000;}";
    html += ".direction{font-size:28px;font-weight:bold;margin:20px;padding:20px;background:#ffeb3b;border-radius:10px;text-align:center;}";
    html += ".api-info{margin-top:30px;padding:15px;background:#f5f5f5;border-radius:5px;}";
    html += "code{background:#ddd;padding:2px 6px;border-radius:3px;}";
    html += "</style>";
    html += "<script>";
    html += "function updateData(){";
    html += "  fetch('/data').then(r=>r.json()).then(d=>{";
    html += "    document.getElementById('right-dist').textContent=d.right.valid?d.right.distance+'mm':'ERROR ('+d.right.status+')';";
    html += "    document.getElementById('right-dist').className=d.right.valid?'distance':'distance invalid';";
    html += "    document.getElementById('front-dist').textContent=d.front.valid?d.front.distance+'mm':'ERROR ('+d.front.status+')';";
    html += "    document.getElementById('front-dist').className=d.front.valid?'distance':'distance invalid';";
    html += "    document.getElementById('left-dist').textContent=d.left.valid?d.left.distance+'mm':'ERROR ('+d.left.status+')';";
    html += "    document.getElementById('left-dist').className=d.left.valid?'distance':'distance invalid';";
    html += "    document.getElementById('direction').textContent=d.direction;";
    html += "  });";
    html += "}";
    html += "setInterval(updateData,100);";
    html += "updateData();";
    html += "</script>";
    html += "</head><body>";
    html += "<div class='container'>";
    html += "<h1>🤖 Bottle Sumo Robot</h1>";
    html += "<h2 style='text-align:center;color:#666;'>ToF Sensor Monitor</h2>";
    html += "<div style='text-align:center;'>";
    html += "<div class='sensor'><h3>RIGHT</h3><div id='right-dist' class='distance'>---</div></div>";
    html += "<div class='sensor'><h3>FRONT</h3><div id='front-dist' class='distance'>---</div></div>";
    html += "<div class='sensor'><h3>LEFT</h3><div id='left-dist' class='distance'>---</div></div>";
    html += "</div>";
    html += "<div class='direction'><div style='font-size:14px;color:#666;'>DETECTION</div><div id='direction'>CLEAR</div></div>";
    html += "<div class='api-info'>";
    html += "<h3>API Endpoints:</h3>";
    html += "<p><code>GET /data</code> - Returns JSON with all sensor readings</p>";
    html += "<p><code>GET /json</code> - Same as /data (alias)</p>";
    html += "<p>Update rate: ~10Hz (100ms)</p>";
    html += "</div>";
    html += "</div>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
  });
  
  // JSON data endpoint
  server.on("/data", HTTP_GET, []() {
    String json = get_json_reading(latest_reading);
    server.sendHeader("Access-Control-Allow-Origin", "*"); // Allow CORS
    server.send(200, "application/json", json);
  });
  
  // Alias for /data
  server.on("/json", HTTP_GET, []() {
    String json = get_json_reading(latest_reading);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", json);
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
  
  // 404 handler
  server.onNotFound([]() {
    server.send(404, "text/plain", "404: Not Found");
  });
  
  server.begin();
  Serial.println("HTTP server started!");
  Serial.println("Available endpoints:");
  Serial.println("  /       - Web dashboard");
  Serial.println("  /data   - JSON sensor data");
  Serial.println("  /json   - JSON sensor data (alias)");
  Serial.println("  /status - WiFi status info");
}

// ============== Setup & Loop ==============

void setup() {
  Serial.begin(115200);
  // Wait for Serial with timeout (3 seconds) for GUI debugging
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000));

  Serial.println("\n\n=== Bottle Sumo Robot - WiFi AP Mode ===");

  // Initialize I2C
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();

  // Initialize ToF sensors
  Serial.println("\nInitializing ULTRA FAST ToF sensors...");
  if (!init_tof_sensors()) {
    Serial.println("ToF sensor initialization failed!");
    while (1);
  }
  
  Serial.println("ToF sensors ready! ULTRA FAST mode");
  scanI2C(); // Debug: Verify all 3 sensors have unique I2C addresses (0x30, 0x31, 0x32)
  
  // Initialize WiFi AP
  setupWiFiAP();
  
  // Start web server
  setupWebServer();
  
  Serial.println("\n=== System Ready ===");
  Serial.println("Starting sensor readings...\n");
}

void loop() {
  // Handle web server requests
  server.handleClient();
  
  // Read sensors
  latest_reading = read_tof_sensors();
  
  // Print to Serial for debugging
  print_tof_readings(latest_reading);
  
  String direction = get_object_direction(latest_reading, DETECTION_THRESHOLD_MM);
  if (direction != "CLEAR") {
    Serial.print("DETECTED: ");
    Serial.println(direction);
  }
  
  delay(LOOP_DELAY_MS); // TARGET: ~33 readings per second!
}
