/**************************************************************************
 Pico W Access Point Mode Test with OLED Display
 
 This sketch creates a WiFi Access Point and displays the connection
 information on an OLED display. No sensors - just AP mode testing.
 
 Hardware:
 - Raspberry Pi Pico W
 - SSD1306 OLED Display (128x64) on I2C
 - I2C: GP26 (SDA), GP27 (SCL)
 **************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

// WiFi AP Configuration
const char* ap_ssid = "BottleSumo_Robot";      // WiFi network name
const char* ap_password = "sumo2025";          // Password (min 8 characters)

WebServer server(80);

// Global variables for status tracking
bool ap_active = false;
IPAddress ap_ip(192, 168, 4, 1);  // Custom IP address
int client_count = 0;

// Simulated ToF sensor data (placeholder until sensors connected)
struct ToF_Data {
  uint16_t right_distance = 0;
  uint16_t front_distance = 0;
  uint16_t left_distance = 0;
  bool sensors_connected = false;
} tof_data;

// ============== OLED Display Functions ==============

void initOLED() {
  Serial.println("Initializing OLED display...");
  
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    // Show error on serial but continue
    return;
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Pico W AP Test"));
  display.println(F("Initializing..."));
  display.display();
  
  Serial.println("OLED initialized successfully!");
}

void updateOLEDDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Title (smaller)
  display.setCursor(0, 0);
  display.println(F("PICO W AP SERVER"));
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  if (ap_active) {
    // SSID (compact)
    display.setCursor(0, 11);
    display.print(F("WiFi:"));
    display.println(ap_ssid);
    
    // IP Address (compact)
    display.setCursor(0, 20);
    display.print(F("IP:"));
    display.println(ap_ip);
    
    // Separator line
    display.drawLine(0, 29, 128, 29, SSD1306_WHITE);
    
    // ToF Sensor Data Section
    display.setCursor(0, 31);
    display.print(F("ToF Sensors:"));
    
    if (tof_data.sensors_connected) {
      // Show actual sensor data
      display.setCursor(0, 40);
      display.print(F("R:"));
      display.print(tof_data.right_distance);
      display.print(F(" F:"));
      display.print(tof_data.front_distance);
      display.print(F(" L:"));
      display.println(tof_data.left_distance);
    } else {
      // Show "Not Connected"
      display.setCursor(0, 40);
      display.println(F("Status: Waiting..."));
      display.setCursor(0, 49);
      display.print(F("R:--- F:--- L:---"));
    }
    
    // Status bar at bottom
    display.drawLine(0, 57, 128, 57, SSD1306_WHITE);
    display.setCursor(0, 58);
    display.print(F("Cl:"));
    display.print(client_count);
    display.print(F(" Up:"));
    display.print(millis()/1000);
    display.print(F("s"));
    
  } else {
    // Error message
    display.setCursor(20, 30);
    display.setTextSize(2);
    display.println(F("AP"));
    display.setCursor(10, 48);
    display.println(F("FAILED!"));
  }
  
  display.display();
}

// ============== WiFi AP Functions ==============

void setupWiFiAP() {
  Serial.println("\n=== Setting up WiFi Access Point ===");
  
  // Configure Access Point with custom IP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ap_ip, ap_ip, IPAddress(255, 255, 255, 0));
  ap_active = WiFi.softAP(ap_ssid, ap_password);
  
  if (ap_active) {
    Serial.println("Access Point created successfully!");
    Serial.print("SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Password: ");
    Serial.println(ap_password);
    Serial.print("AP IP address: ");
    Serial.println(ap_ip);
    Serial.println("\nConnect your device to this WiFi network");
    Serial.print("Then access: http://");
    Serial.println(ap_ip);
  } else {
    Serial.println("FAILED to create Access Point!");
  }
  
  // Update OLED with initial status
  updateOLEDDisplay();
}


// ============== Web Server Functions ==============

void setupWebServer() {
  // Root page - Simple status page
  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
    html += "<title>Pico W AP Test</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>";
    html += "body{font-family:Arial,sans-serif;margin:20px;background:#f0f0f0;text-align:center;}";
    html += ".container{max-width:600px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);}";
    html += "h1{color:#0066cc;margin-bottom:10px;}";
    html += ".status{font-size:18px;margin:20px 0;padding:20px;background:#e8f4f8;border-radius:5px;}";
    html += ".sensor-box{margin:20px 0;padding:15px;background:#f5f5f5;border-radius:5px;}";
    html += ".sensor-data{display:inline-block;width:30%;margin:5px;padding:10px;background:#fff;border:2px solid #0066cc;border-radius:5px;}";
    html += ".sensor-label{font-size:12px;color:#666;font-weight:bold;}";
    html += ".sensor-value{font-size:24px;color:#0066cc;font-weight:bold;margin:5px 0;}";
    html += ".info{font-size:16px;margin:10px 0;color:#333;}";
    html += ".success{color:#00aa00;font-weight:bold;}";
    html += ".waiting{color:#ff9900;font-weight:bold;}";
    html += "</style>";
    html += "<script>";
    html += "function updateData(){";
    html += "  fetch('/data').then(r=>r.json()).then(d=>{";
    html += "    document.getElementById('right-val').textContent=d.sensors_connected?d.right+'mm':'---';";
    html += "    document.getElementById('front-val').textContent=d.sensors_connected?d.front+'mm':'---';";
    html += "    document.getElementById('left-val').textContent=d.sensors_connected?d.left+'mm':'---';";
    html += "    document.getElementById('sensor-status').textContent=d.sensors_connected?'Connected':'Waiting...';";
    html += "    document.getElementById('sensor-status').className=d.sensors_connected?'success':'waiting';";
    html += "    document.getElementById('clients').textContent=d.clients;";
    html += "    document.getElementById('uptime').textContent=d.uptime_seconds;";
    html += "  }).catch(e=>console.error('Error:',e));";
    html += "}";
    html += "setInterval(updateData,500);";  // Update every 500ms
    html += "updateData();";
    html += "</script>";
    html += "</head><body>";
    html += "<div class='container'>";
    html += "<h1>🤖 Pico W AP Test</h1>";
    html += "<div class='status'>";
    html += "<div class='success'>✓ Access Point Active</div>";
    html += "</div>";
    html += "<div class='info'><strong>SSID:</strong> " + String(ap_ssid) + "</div>";
    html += "<div class='info'><strong>IP:</strong> " + ap_ip.toString() + "</div>";
    html += "<div class='sensor-box'>";
    html += "<h3 style='margin-top:0;'>ToF Sensors</h3>";
    html += "<div id='sensor-status' class='waiting'>Waiting...</div>";
    html += "<div style='margin-top:15px;'>";
    html += "<div class='sensor-data'><div class='sensor-label'>RIGHT</div><div id='right-val' class='sensor-value'>---</div></div>";
    html += "<div class='sensor-data'><div class='sensor-label'>FRONT</div><div id='front-val' class='sensor-value'>---</div></div>";
    html += "<div class='sensor-data'><div class='sensor-label'>LEFT</div><div id='left-val' class='sensor-value'>---</div></div>";
    html += "</div></div>";
    html += "<div class='info'><strong>Clients:</strong> <span id='clients'>" + String(WiFi.softAPgetStationNum()) + "</span></div>";
    html += "<div class='info'><strong>Uptime:</strong> <span id='uptime'>" + String(millis()/1000) + "</span>s</div>";
    html += "<hr style='margin:30px 0;'>";
    html += "<h3>API Endpoints:</h3>";
    html += "<div class='info'><code>GET /data</code> - Sensor data JSON</div>";
    html += "<div class='info'><code>GET /status</code> - Status JSON</div>";
    html += "</div>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
  });
  
  // Data endpoint - ToF sensor data
  server.on("/data", HTTP_GET, []() {
    String json = "{";
    json += "\"right\":" + String(tof_data.right_distance) + ",";
    json += "\"front\":" + String(tof_data.front_distance) + ",";
    json += "\"left\":" + String(tof_data.left_distance) + ",";
    json += "\"sensors_connected\":" + String(tof_data.sensors_connected ? "true" : "false") + ",";
    json += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
    json += "\"uptime_ms\":" + String(millis()) + ",";
    json += "\"uptime_seconds\":" + String(millis()/1000);
    json += "}";
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", json);
  });
  
  // Status endpoint - JSON format
  server.on("/status", HTTP_GET, []() {
    String json = "{";
    json += "\"wifi_ssid\":\"" + String(ap_ssid) + "\",";
    json += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
    json += "\"connected_clients\":" + String(WiFi.softAPgetStationNum()) + ",";
    json += "\"uptime_ms\":" + String(millis()) + ",";
    json += "\"uptime_seconds\":" + String(millis()/1000);
    json += "}";
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", json);
  });
  
  // Info endpoint - System info
  server.on("/info", HTTP_GET, []() {
    String json = "{";
    json += "\"device\":\"Raspberry Pi Pico W\",";
    json += "\"mode\":\"Access Point\",";
    json += "\"ssid\":\"" + String(ap_ssid) + "\",";
    json += "\"ip\":\"" + ap_ip.toString() + "\",";
    json += "\"mac\":\"" + WiFi.macAddress() + "\",";
    json += "\"clients\":" + String(WiFi.softAPgetStationNum());
    json += "}";
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", json);
  });
  
  // 404 handler
  server.onNotFound([]() {
    server.send(404, "text/plain", "404: Not Found\n\nAvailable endpoints:\n  /       - Web page\n  /data   - Sensor data JSON\n  /status - Status JSON\n  /info   - System info JSON");
  });
  
  server.begin();
  Serial.println("HTTP server started!");
  Serial.println("Available endpoints:");
  Serial.println("  /       - Web page");
  Serial.println("  /data   - Sensor data JSON");
  Serial.println("  /status - Status JSON");
  Serial.println("  /info   - System info JSON");
}

// ============== Setup & Loop ==============

void setup() {
  Serial.begin(115200);
  // Wait for Serial with timeout (3 seconds)
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000));
  
  Serial.println("\n\n=== Pico W Access Point Test ===");
  Serial.println("Testing WiFi AP + OLED Display");
  
  // Initialize OLED display first
  initOLED();
  delay(1000);
  
  // Initialize WiFi AP
  setupWiFiAP();
  delay(500);
  
  // Start web server
  if (ap_active) {
    setupWebServer();
  }
  
  Serial.println("\n=== System Ready ===");
  Serial.println("OLED will update every second\n");
}

void loop() {
  // Handle web server requests
  if (ap_active) {
    server.handleClient();
  }
  
  // Update client count
  client_count = WiFi.softAPgetStationNum();
  
  // Simulate sensor data changes (remove this when real sensors are connected)
  // This makes the display show changing values for testing
  static unsigned long lastSensorUpdate = 0;
  if (millis() - lastSensorUpdate >= 200) {  // Update every 200ms
    lastSensorUpdate = millis();
    if (!tof_data.sensors_connected) {
      // Simulate "searching" animation with dashes
      static int counter = 0;
      counter++;
      // Keep at 0 to show "---" until real sensors connect
    }
  }
  
  // Update OLED display every second
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();
    updateOLEDDisplay();
    
    // Print status to Serial
    Serial.print("AP Status - Clients: ");
    Serial.print(client_count);
    Serial.print(" | Uptime: ");
    Serial.print(millis()/1000);
    Serial.print("s | Sensors: ");
    Serial.println(tof_data.sensors_connected ? "Connected" : "Waiting...");
  }
  
  delay(10); // Small delay to prevent watchdog issues
}
