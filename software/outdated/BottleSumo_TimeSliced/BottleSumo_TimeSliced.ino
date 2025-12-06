/*
 * Bottle Sumo Robot - Time-Sliced Dual-Core Architecture
 * Raspberry Pi Pico W with strict time budget enforcement
 * 
 * NEW ARCHITECTURE (2025-10-13):
 * ==================================
 * Core 0: Pure state machine
 *   - Reads sensor data from shared memory
 *   - Executes decision logic
 *   - Outputs motor commands to shared memory
 * 
 * Core 1: ALL I/O operations with time-sliced scheduling
 *   - 10ms: ADS1115 IR sensor read
 *   - 250ms: ToF sensors (5 sensors @ 50ms each)
 *   - 10ms: ADS1115 IR sensor read
 *   - 5ms: Button sampling
 *   - 20ms: Button debouncing
 *   - 10ms: ADS1115 IR sensor read
 *   - 5ms: Motor PWM update
 *   - 50ms: WiFi/TCP server handling
 *   - 10ms: ADS1115 IR sensor read
 *   Total cycle: ~370ms (~2.7Hz)
 * 
 * Hardware:
 * - ADS1115 on Wire (IR sensors)
 * - 5x VL53L0X ToF on Wire1
 * - 2x Buttons (TEST_MODE: GP15, RUN_MODE: GP16)
 * - WiFi AP mode, TCP port 4242
 * 
 * OLED REMOVED (no longer needed)
 */

#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>

// Custom modules
#include "ToFArray.h"
#include "Ads1115Sampler.h"
#include "ButtonManager.h"
#include "Motor.h"
#include "TestModeCommon.h"
#include "MotorTestMode.h"
#include "SensorTestMode.h"

// Forward declarations so Arduino's auto-prototyper recognizes custom types
struct SensorData;
struct MotorCommand;
enum SumoAction : uint8_t;
struct EdgeDetection;

// ========== Hardware Configuration ==========

//I2C pin
constexpr uint8_t I2C_SDA = 2;
constexpr uint8_t I2C_SCL = 3;

// Button pins
constexpr uint8_t BUTTON_TEST_MODE_PIN = 18;  // GP18
constexpr uint8_t BUTTON_RUN_MODE_PIN = 28;   // GP28

// ToF sensor configuration (5 sensors)
constexpr uint8_t TOF_XSHUT_PINS[5] = {4, 5, 6, 7, 8}; // GP12, GP11, GP13, GP10, GP14
constexpr uint8_t TOF_ADDRESSES[5] = {0x30, 0x31, 0x32, 0x33, 0x34};
constexpr uint32_t TOF_TIMING_BUDGET_US = 50000; // 50ms per sensor for long range stable
constexpr uint8_t TOF_VCSEL_PRE_RANGE = 14;
constexpr uint8_t TOF_VCSEL_FINAL_RANGE = 10;
constexpr uint16_t TOF_MAX_RANGE_MM = 1500;
constexpr uint16_t TOF_MIN_RANGE_MM = 30;
constexpr uint8_t TOF_MAX_STATUS = 2;

// ADS1115 configuration
constexpr uint8_t ADS1115_ADDRESS = 0x48;
constexpr uint8_t IR_SENSOR_COUNT = 4;

// Motor configuration
constexpr uint8_t MOTOR_LEFT_PWM_PIN = 11;   // GP6
constexpr uint8_t MOTOR_LEFT_DIR_PIN = 12;   // GP7
constexpr uint8_t MOTOR_RIGHT_PWM_PIN = 14;  // GP8
constexpr uint8_t MOTOR_RIGHT_DIR_PIN = 15;  // GP9
constexpr uint32_t MOTOR_PWM_FREQ = 20000;  // 20kHz PWM frequency

// WiFi configuration
constexpr const char* AP_SSID = "BottleSumo_Robot";
constexpr const char* AP_PASSWORD = "sumo2025";
constexpr uint16_t TCP_PORT = 4242;
constexpr uint8_t MAX_CLIENTS = 4;

// I2C helper for diagnostics
uint8_t detectI2CDevice(TwoWire &bus, const uint8_t *candidates, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    bus.beginTransmission(candidates[i]);
    if (bus.endTransmission() == 0) {
      return candidates[i];
    }
  }
  return 0;
}

void scanI2CBus(TwoWire &bus, const char *label) {
  Serial.printf("%s bus scan: ", label);
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    bus.beginTransmission(address);
    if (bus.endTransmission() == 0) {
      Serial.printf("0x%02X ", address);
      found++;
    }
  }
  if (!found) Serial.print("none");
  Serial.printf("(%d found)\n", found);
}

// Time budgets (milliseconds)
constexpr uint32_t BUDGET_ADS_READ = 10;
constexpr uint32_t BUDGET_TOF_READ = 250;  // 5 sensors @ 50ms each
constexpr uint32_t BUDGET_BUTTON_SAMPLE = 5;
constexpr uint32_t BUDGET_BUTTON_DEBOUNCE = 20;
constexpr uint32_t BUDGET_WIFI_HANDLE = 50;
constexpr uint32_t BUDGET_MOTOR_PWM = 5;  // Motor PWM update budget

// Sensor thresholds
constexpr float EDGE_THRESHOLD_DEFAULT = 2.5f;
constexpr float EDGE_THRESHOLD_MIN = 0.1f;
constexpr float EDGE_THRESHOLD_MAX = 4.0f;

// ========== Shared Data Structures ==========

// Sensor data (Core 1 writes, Core 0 reads)
struct SensorData {
  // IR sensors
  int16_t ir_raw[IR_SENSOR_COUNT];
  float ir_volts[IR_SENSOR_COUNT];
  unsigned long ir_timestamp;
  
  // ToF sensors
  uint16_t tof_distance[5];
  bool tof_valid[5];
  uint8_t tof_status[5];
  unsigned long tof_timestamp;
  
  // Button state
  ButtonMode button_mode;
  bool test_button_pressed;
  bool run_button_pressed;
  
  // Data ready flags
  bool ir_ready;
  bool tof_ready;
};

// Motor commands (Core 0 writes, Core 1 reads for PWM)
struct MotorCommand {
  int8_t left_speed;    // -100 to +100 (duty cycle percentage)
  int8_t right_speed;   // -100 to +100 (duty cycle percentage)
  bool emergency_stop;
  unsigned long timestamp;
};

// Sumo robot state with directional edge detection
enum SumoAction : uint8_t {
  SEARCH_OPPONENT,
  ATTACK_FORWARD,
  
  // Edge detection retreat actions (directional)
  RETREAT_FORWARD,          // Both bottom sensors (A2+A3) - move forward away from rear edge
  RETREAT_BACKWARD,         // Both top sensors (A0+A1) - move backward away from front edge
  RETREAT_FORWARD_LEFT,     // Bottom-right sensor (A3) - diagonal retreat
  RETREAT_FORWARD_RIGHT,    // Bottom-left sensor (A2) - diagonal retreat
  RETREAT_BACKWARD_LEFT,    // Top-right sensor (A1) - diagonal retreat
  RETREAT_BACKWARD_RIGHT,   // Top-left sensor (A0) - diagonal retreat
  RETREAT_LEFT,             // Right side sensors (A1+A3) - strafe left
  RETREAT_RIGHT,            // Left side sensors (A0+A2) - strafe right
  
  EMERGENCY_STOP,           // All sensors or critical failure
  IDLE                      // Test mode or stopped
};
struct EdgeDetection {
  uint8_t pattern;      // Bitfield of detected edges
  uint8_t count;        // Number of edges detected
  bool top_left;        // A0 - front-left corner
  bool top_right;       // A1 - front-right corner
  bool bottom_left;     // A2 - rear-left corner
  bool bottom_right;    // A3 - rear-right corner
};
// ========== Global Objects & State ==========

// Mutexes for thread safety
mutex_t data_mutex;      // Protects SensorData
mutex_t motor_mutex;     // Protects MotorCommand
mutex_t wire1_mutex;     // Protects Wire1 I2C bus (ToF sensors)
mutex_t threshold_mutex; // Protects runtime thresholds

// Shared data
volatile SensorData shared_sensor_data;
MotorCommand shared_motor_cmd;
volatile float runtime_thresholds[IR_SENSOR_COUNT] = {
  EDGE_THRESHOLD_DEFAULT, EDGE_THRESHOLD_DEFAULT, 
  EDGE_THRESHOLD_DEFAULT, EDGE_THRESHOLD_DEFAULT
};

// Core 0 state tracking
volatile SumoAction current_action = IDLE;
EdgeDetection current_edges = {0, 0, false, false, false, false};

// Core 1 objects
Ads1115Sampler ads_sampler;
ToFArray tof_array(&Wire1, &wire1_mutex);
ButtonManager buttons(BUTTON_TEST_MODE_PIN, BUTTON_RUN_MODE_PIN, 20);
WiFiServer tcp_server(TCP_PORT);
Motor motor_left(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN);
Motor motor_right(MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN);

// Test mode state
TestModeState test_mode_state;

// Core status
volatile bool core1_ready = false;
volatile unsigned long core0_loop_count = 0;
volatile unsigned long core1_loop_count = 0;

// WiFi state
struct ClientState {
  WiFiClient client;
  bool active;
  unsigned long last_activity;
};
ClientState clients[MAX_CLIENTS];

// ========== Helper Functions ==========

// Edge detection bitfield (for fast pattern matching)
// Bit 0 = A0 (top-left), Bit 1 = A1 (top-right), Bit 2 = A2 (bottom-left), Bit 3 = A3 (bottom-right)


// Analyze edge sensor pattern
EdgeDetection analyzeEdges(const SensorData &sensors) {
  EdgeDetection result = {0, 0, false, false, false, false};
  
  // Check each sensor against threshold
  result.top_left = sensors.ir_volts[0] > getThreshold(0);      // A0
  result.top_right = sensors.ir_volts[1] > getThreshold(1);     // A1
  result.bottom_left = sensors.ir_volts[2] > getThreshold(2);   // A2
  result.bottom_right = sensors.ir_volts[3] > getThreshold(3);  // A3
  
  // Build bitfield pattern
  result.pattern = 0;
  if (result.top_left)     result.pattern |= 0b0001;
  if (result.top_right)    result.pattern |= 0b0010;
  if (result.bottom_left)  result.pattern |= 0b0100;
  if (result.bottom_right) result.pattern |= 0b1000;
  
  // Count edges
  result.count = result.top_left + result.top_right + result.bottom_left + result.bottom_right;
  
  return result;
}

// Convert SumoAction to string for debugging/telemetry
const char* actionToString(SumoAction action) {
  switch (action) {
    case SEARCH_OPPONENT: return "SEARCH";
    case ATTACK_FORWARD: return "ATTACK";
    case RETREAT_FORWARD: return "RETREAT_FWD";
    case RETREAT_BACKWARD: return "RETREAT_BACK";
    case RETREAT_FORWARD_LEFT: return "RETREAT_FWD_LEFT";
    case RETREAT_FORWARD_RIGHT: return "RETREAT_FWD_RIGHT";
    case RETREAT_BACKWARD_LEFT: return "RETREAT_BACK_LEFT";
    case RETREAT_BACKWARD_RIGHT: return "RETREAT_BACK_RIGHT";
    case RETREAT_LEFT: return "RETREAT_LEFT";
    case RETREAT_RIGHT: return "RETREAT_RIGHT";
    case EMERGENCY_STOP: return "EMERGENCY_STOP";
    case IDLE: return "IDLE";
    default: return "UNKNOWN";
  }
}

// Thread-safe threshold access
float getThreshold(int index) {
  if (index < 0 || index >= IR_SENSOR_COUNT) return EDGE_THRESHOLD_DEFAULT;
  mutex_enter_blocking(&threshold_mutex);
  float val = runtime_thresholds[index];
  mutex_exit(&threshold_mutex);
  return val;
}

bool setThreshold(int index, float value) {
  if (index < 0 || index >= IR_SENSOR_COUNT) return false;
  if (value < EDGE_THRESHOLD_MIN || value > EDGE_THRESHOLD_MAX) return false;
  mutex_enter_blocking(&threshold_mutex);
  runtime_thresholds[index] = value;
  mutex_exit(&threshold_mutex);
  return true;
}

// Initialize Wire1 for ToF sensors
void initWire1() {
  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);  
  Wire1.begin();
  Wire1.setClock(400000);
}

// ========== Core 1: I/O Hub with Time-Sliced Scheduler ==========

void setup1() {
  delay(100); // Wait for Core 0 basic init
  
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("Core 1: I/O Hub Initializing...");
  
  // Initialize I2C buses
  Wire.begin();
  Wire.setClock(400000);
  initWire1();
  scanI2CBus(Wire, "Wire");
  scanI2CBus(Wire1, "Wire1");
  
  // Initialize ADS1115 on Wire1 (same bus as ToF sensors)
  const uint8_t adsCandidates[] = {0x48, 0x49, 0x4A, 0x4B};
  uint8_t detectedAddress = detectI2CDevice(Wire1, adsCandidates, sizeof(adsCandidates));
  if (!detectedAddress) {
    Serial.println("ERROR: ADS1115 not detected on Wire1 bus");
    while(1) delay(1000);
  }

  if (!ads_sampler.begin(detectedAddress, &Wire1, GAIN_ONE, RATE_ADS1115_860SPS)) {
    Serial.println("ERROR: ADS1115 init failed");
    while(1) delay(1000);
  }
  Serial.printf("✓ ADS1115 ready at 0x%02X on Wire1\n", detectedAddress);
  
  // Initialize ToF array
  if (!tof_array.configure(5, TOF_XSHUT_PINS, TOF_ADDRESSES)) {
    Serial.println("ERROR: ToF config failed");
  }
  tof_array.setTiming(TOF_TIMING_BUDGET_US, TOF_VCSEL_PRE_RANGE, TOF_VCSEL_FINAL_RANGE);
  uint8_t tof_ok = tof_array.beginAll();
  Serial.printf("✓ ToF sensors: %d/5 online\n", tof_ok);
  
  // Initialize buttons
  buttons.begin();
  Serial.println("✓ Buttons ready");
  
  // Initialize motors
  motor_left.begin(MOTOR_PWM_FREQ);
  motor_right.begin(MOTOR_PWM_FREQ);
  motor_left.stop();
  motor_right.stop();
  Serial.println("✓ Motors ready");
  
  // Initialize WiFi AP
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("ERROR: WiFi AP failed");
  } else {
    Serial.printf("✓ WiFi AP: %s @ %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  }
  
  // Start TCP server
  tcp_server.begin();
  Serial.printf("✓ TCP server on port %d\n", TCP_PORT);
  
  for (int i = 0; i < MAX_CLIENTS; i++) {
    clients[i].active = false;
  }
  
  core1_ready = true;
  Serial.println("Core 1: Ready. Starting time-sliced scheduler...\n");
}

// Time-sliced main loop for Core 1
void loop1() {
  core1_loop_count++;
  uint32_t cycle_start = millis();
  
  // ===== TASK 1: ADS Read (10ms budget) =====
  uint32_t t_start = millis();
  int16_t raw[IR_SENSOR_COUNT];
  float volts[IR_SENSOR_COUNT];
  ads_sampler.readAll(raw, volts, IR_SENSOR_COUNT);
  
  mutex_enter_blocking(&data_mutex);
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    shared_sensor_data.ir_raw[i] = raw[i];
    shared_sensor_data.ir_volts[i] = volts[i];
  }
  shared_sensor_data.ir_timestamp = millis();
  shared_sensor_data.ir_ready = true;
  mutex_exit(&data_mutex);
  
  uint32_t t_elapsed = millis() - t_start;
  if (t_elapsed > BUDGET_ADS_READ) {
    Serial.printf("⚠️ ADS1 overrun: %lums (budget: %lums)\n", t_elapsed, BUDGET_ADS_READ);
  }
  
  // ===== TASK 2: ToF Read (150ms budget) =====
  t_start = millis();
  ToFSample tof_samples[5];
  tof_array.readAll(tof_samples, TOF_MIN_RANGE_MM, TOF_MAX_RANGE_MM, TOF_MAX_STATUS);
  
  mutex_enter_blocking(&data_mutex);
  for (int i = 0; i < 5; i++) {
    shared_sensor_data.tof_distance[i] = tof_samples[i].distanceMm;
    shared_sensor_data.tof_valid[i] = tof_samples[i].valid;
    shared_sensor_data.tof_status[i] = tof_samples[i].status;
  }
  shared_sensor_data.tof_timestamp = millis();
  shared_sensor_data.tof_ready = true;
  mutex_exit(&data_mutex);
  
  t_elapsed = millis() - t_start;
  if (t_elapsed > BUDGET_TOF_READ) {
    Serial.printf("⚠️ ToF overrun: %lums (budget: %lums)\n", t_elapsed, BUDGET_TOF_READ);
  }
  
  // ===== TASK 3: ADS Read (10ms budget) =====
  t_start = millis();
  ads_sampler.readAll(raw, volts, IR_SENSOR_COUNT);
  
  mutex_enter_blocking(&data_mutex);
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    shared_sensor_data.ir_raw[i] = raw[i];
    shared_sensor_data.ir_volts[i] = volts[i];
  }
  shared_sensor_data.ir_timestamp = millis();
  mutex_exit(&data_mutex);
  
  t_elapsed = millis() - t_start;
  if (t_elapsed > BUDGET_ADS_READ) {
    Serial.printf("⚠️ ADS2 overrun: %lums\n", t_elapsed);
  }
  
  // ===== TASK 4: Button Sample (5ms budget) =====
  t_start = millis();
  bool sample_ok = buttons.sample(BUDGET_BUTTON_SAMPLE);
  if (!sample_ok) {
    Serial.println("⚠️ Button sample overrun");
  }
  
  // ===== TASK 5: Button Debounce (20ms budget) =====
  t_start = millis();
  bool debounce_ok = buttons.debounce(BUDGET_BUTTON_DEBOUNCE);
  if (!debounce_ok) {
    Serial.println("⚠️ Button debounce overrun");
  }
  
  mutex_enter_blocking(&data_mutex);
  shared_sensor_data.button_mode = buttons.getMode();
  shared_sensor_data.test_button_pressed = buttons.testButtonPressed();
  shared_sensor_data.run_button_pressed = buttons.runButtonPressed();
  mutex_exit(&data_mutex);
  
  // ===== TASK 6: ADS Read (10ms budget) =====
  t_start = millis();
  ads_sampler.readAll(raw, volts, IR_SENSOR_COUNT);
  
  mutex_enter_blocking(&data_mutex);
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    shared_sensor_data.ir_raw[i] = raw[i];
    shared_sensor_data.ir_volts[i] = volts[i];
  }
  shared_sensor_data.ir_timestamp = millis();
  mutex_exit(&data_mutex);
  
  t_elapsed = millis() - t_start;
  if (t_elapsed > BUDGET_ADS_READ) {
    Serial.printf("⚠️ ADS3 overrun: %lums\n", t_elapsed);
  }
  
  // ===== TASK 7: Motor PWM Update (5ms budget) =====
  t_start = millis();
  
  // Read motor commands from Core 0 or test mode
  MotorCommand local_motor_cmd;
  mutex_enter_blocking(&motor_mutex);
  local_motor_cmd = shared_motor_cmd;
  mutex_exit(&motor_mutex);
  
  // Apply motor commands
  if (local_motor_cmd.emergency_stop || test_mode_state.motor.emergency_stop) {
    motor_left.stop();
    motor_right.stop();
  } else if (test_mode_state.mode == TestMode::TEST_MOTOR) {
    // Test mode takes priority
    motor_left.setDuty(test_mode_state.motor.left_pwm);
    motor_right.setDuty(test_mode_state.motor.right_pwm);
  } else {
    // Normal operation mode (from Core 0 decision logic)
    motor_left.setDuty(local_motor_cmd.left_speed);
    motor_right.setDuty(local_motor_cmd.right_speed);
  }
  
  t_elapsed = millis() - t_start;
  if (t_elapsed > BUDGET_MOTOR_PWM) {
    Serial.printf("⚠️ Motor PWM overrun: %lums\n", t_elapsed);
  }
  
  // ===== TASK 8: WiFi/TCP Handle (50ms budget) =====
  t_start = millis();
  handleWiFiTCP();
  t_elapsed = millis() - t_start;
  if (t_elapsed > BUDGET_WIFI_HANDLE) {
    Serial.printf("⚠️ WiFi overrun: %lums (budget: %lums)\n", t_elapsed, BUDGET_WIFI_HANDLE);
  }
  
  // ===== TASK 9: ADS Read (10ms budget) =====
  t_start = millis();
  ads_sampler.readAll(raw, volts, IR_SENSOR_COUNT);
  
  mutex_enter_blocking(&data_mutex);
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    shared_sensor_data.ir_raw[i] = raw[i];
    shared_sensor_data.ir_volts[i] = volts[i];
  }
  shared_sensor_data.ir_timestamp = millis();
  mutex_exit(&data_mutex);
  
  t_elapsed = millis() - t_start;
  if (t_elapsed > BUDGET_ADS_READ) {
    Serial.printf("⚠️ ADS4 overrun: %lums\n", t_elapsed);
  }
  
  // ===== End of cycle timing =====
  uint32_t cycle_time = millis() - cycle_start;
  
  // Log cycle time every 100 cycles
  if (core1_loop_count % 100 == 0) {
    Serial.printf("Core 1 cycle: %lums (target: ~370ms) | Loop: %lu\n", 
                  cycle_time, core1_loop_count);
  }
}

// WiFi/TCP handler (non-blocking, 50ms budget)
void handleWiFiTCP() {
  uint32_t start = millis();
  
  // Accept new clients (quick check)
  WiFiClient new_client = tcp_server.accept();
  if (new_client) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
      if (!clients[i].active) {
        clients[i].client = new_client;
        clients[i].active = true;
        clients[i].last_activity = millis();
        Serial.printf("Client %d connected\n", i);
        new_client.println("{\"status\":\"connected\"}");
        break;
      }
    }
  }
  
  // Check budget before processing clients
  if (millis() - start > BUDGET_WIFI_HANDLE / 2) return;
  
  // Service existing clients (time-boxed)
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (!clients[i].active) continue;
    
    WiFiClient &c = clients[i].client;
    
    if (!c.connected()) {
      c.stop();
      clients[i].active = false;
      Serial.printf("Client %d disconnected\n", i);
      continue;
    }
    
    // Handle commands (simplified, no blocking reads)
    if (c.available()) {
      String cmd = c.readStringUntil('\n');
      cmd.trim();
      
      String response = "";
      
      // Threshold command: "threshold,0,2.5" sets sensor 0 to 2.5V
      if (cmd.startsWith("threshold,")) {
        int idx = cmd.substring(10, cmd.indexOf(',', 10)).toInt();
        float val = cmd.substring(cmd.lastIndexOf(',') + 1).toFloat();
        if (setThreshold(idx, val)) {
          c.printf("{\"ack\":\"threshold_set\",\"sensor\":%d,\"value\":%.2f}\n", idx, val);
        }
      }
      // Test mode commands
      else if (cmd.startsWith("TEST_MOTOR ")) {
        String args = cmd.substring(11);
        response = MotorTestMode::handleTestMotorCommand(args, test_mode_state, motor_left, motor_right);
        c.print(response);
      }
      else if (cmd == "STOP_MOTOR") {
        response = MotorTestMode::handleStopMotorCommand(test_mode_state, motor_left, motor_right);
        c.print(response);
      }
      else if (cmd == "GET_MOTOR") {
        response = MotorTestMode::handleGetMotorCommand(test_mode_state);
        c.print(response);
      }
      else if (cmd.startsWith("TEST_SENSOR ")) {
        String args = cmd.substring(12);
        response = SensorTestMode::handleTestSensorCommand(args, test_mode_state);
        c.print(response);
      }
      else if (cmd.startsWith("SET_MODE ")) {
        String mode_str = cmd.substring(9);
        mode_str.trim();
        mode_str.toUpperCase();
        test_mode_state.mode = TestModeUtils::parseModeString(mode_str);
        test_mode_state.mode_enter_ms = millis();
        c.printf("{\"ack\":\"mode_set\",\"mode\":\"%s\"}\n", TestModeUtils::getModeString(test_mode_state.mode));
      }
      else if (cmd == "GET_MODE") {
        c.printf("{\"mode\":\"%s\",\"uptime_sec\":%lu}\n", 
                TestModeUtils::getModeString(test_mode_state.mode),
                (millis() - test_mode_state.mode_enter_ms) / 1000);
      }
      clients[i].last_activity = millis();
    }
    
    // Check time budget
    if (millis() - start > BUDGET_WIFI_HANDLE) break;
  }
  
  // Stream sensor data (simple JSON, every cycle = ~3.77Hz)
  String json = buildStreamJSON();
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i].active && clients[i].client.availableForWrite() > json.length()) {
      clients[i].client.print(json);
      clients[i].last_activity = millis();
    }
    
    if (millis() - start > BUDGET_WIFI_HANDLE) break;
  }
}

// Build JSON stream payload
String buildStreamJSON() {
  String json = "{\"schema\":\"2.0\",\"ts\":" + String(millis()) + ",";
  
  mutex_enter_blocking(&data_mutex);
  
  // IR sensors
  json += "\"ir\":{\"raw\":[";
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    json += String(shared_sensor_data.ir_raw[i]);
    if (i < IR_SENSOR_COUNT - 1) json += ",";
  }
  json += "],\"volts\":[";
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    json += String(shared_sensor_data.ir_volts[i], 3);
    if (i < IR_SENSOR_COUNT - 1) json += ",";
  }
  json += "]},";
  
  // ToF sensors
  json += "\"tof\":{\"dist\":[";
  for (int i = 0; i < 5; i++) {
    json += String(shared_sensor_data.tof_distance[i]);
    if (i < 4) json += ",";
  }
  json += "],\"valid\":[";
  for (int i = 0; i < 5; i++) {
    json += shared_sensor_data.tof_valid[i] ? "true" : "false";
    if (i < 4) json += ",";
  }
  json += "]},";
  
  // Button state
  json += "\"mode\":";
  if (shared_sensor_data.button_mode == MODE_TEST) json += "\"TEST\"";
  else if (shared_sensor_data.button_mode == MODE_RUN) json += "\"RUN\"";
  else json += "\"UNKNOWN\"";
  
  mutex_exit(&data_mutex);
  
  // Test mode and motor status
  json += ",\"test_mode\":\"" + String(TestModeUtils::getModeString(test_mode_state.mode)) + "\"";
  json += "," + MotorTestMode::buildMotorJSON(test_mode_state);
  
  // Edge detection and action state (from Core 0)
  json += ",\"robot_state\":{";
  json += "\"action\":\"" + String(actionToString(current_action)) + "\"";
  json += ",\"edges\":{";
  json += "\"count\":" + String(current_edges.count);
  json += ",\"pattern\":" + String(current_edges.pattern);
  json += ",\"A0\":" + String(current_edges.top_left ? "true" : "false");
  json += ",\"A1\":" + String(current_edges.top_right ? "true" : "false");
  json += ",\"A2\":" + String(current_edges.bottom_left ? "true" : "false");
  json += ",\"A3\":" + String(current_edges.bottom_right ? "true" : "false");
  json += "}}";
  
  json += "}\n";
  return json;
}

// ========== Core 0: State Machine ==========

void setup() {
  // Initialize mutexes
  mutex_init(&data_mutex);
  mutex_init(&motor_mutex);
  mutex_init(&wire1_mutex);
  mutex_init(&threshold_mutex);
  
  // Initialize shared data
  memset((void*)&shared_sensor_data, 0, sizeof(SensorData));
  memset((void*)&shared_motor_cmd, 0, sizeof(MotorCommand));
  
  // Wait for Core 1 to complete init
  while (!core1_ready) {
    delay(10);
  }
  
  Serial.println("Core 0: State Machine Ready");
}

void loop() {
  core0_loop_count++;
  
  // Read sensor data
  SensorData local_sensors;
  mutex_enter_blocking(&data_mutex);
  memcpy(&local_sensors, (void*)&shared_sensor_data, sizeof(SensorData));
  mutex_exit(&data_mutex);
  
  // Check data freshness
  if (!local_sensors.ir_ready) {
    delay(5);
    return;
  }
  
  // Analyze edge detection for telemetry
  current_edges = analyzeEdges(local_sensors);
  
  // Run state machine logic
  SumoAction action = decideAction(local_sensors);
  current_action = action;  // Store for telemetry
  
  // Execute action (motor commands)
  MotorCommand cmd = executeAction(action, local_sensors);
  
  // Write motor commands
  mutex_enter_blocking(&motor_mutex);
  shared_motor_cmd = cmd;
  mutex_exit(&motor_mutex);
  
  // Status logging (every 100 loops) with enhanced edge detection info
  if (core0_loop_count % 100 == 0) {
    Serial.printf("Core 0: Action=%s | Edges=%d [A0:%d A1:%d A2:%d A3:%d] | Mode=%d | IR0=%.2fV | ToF0=%dmm | Motors L:%d R:%d\n",
                  actionToString(action),
                  current_edges.count,
                  current_edges.top_left, current_edges.top_right, 
                  current_edges.bottom_left, current_edges.bottom_right,
                  local_sensors.button_mode, 
                  local_sensors.ir_volts[0], 
                  local_sensors.tof_distance[0],
                  cmd.left_speed, cmd.right_speed);
  }
  
  delay(10); // State machine runs at ~100Hz
}

// Decide robot action based on sensor data
SumoAction decideAction(const SensorData &sensors) {
  // Emergency stop if mode unknown
  if (sensors.button_mode != MODE_RUN) {
    return IDLE;
  }
  
  // ===== PRIORITY 1: Edge Detection (highest priority) =====
  EdgeDetection edges = analyzeEdges(sensors);
  
  // Emergency stop if 3 or more edges detected (likely off platform or sensor failure)
  if (edges.count >= 3) {
    return EMERGENCY_STOP;
  }
  
  // Handle edge detection patterns
  if (edges.count > 0) {
    // Pattern matching for directional retreat
    // Patterns use bitfield: [A3 A2 A1 A0]
    
    switch (edges.pattern) {
      // Single sensor detections (corners)
      case 0b0001:  // A0 only (top-left) - retreat backward-right
        return RETREAT_BACKWARD_RIGHT;
        
      case 0b0010:  // A1 only (top-right) - retreat backward-left
        return RETREAT_BACKWARD_LEFT;
        
      case 0b0100:  // A2 only (bottom-left) - retreat forward-right
        return RETREAT_FORWARD_RIGHT;
        
      case 0b1000:  // A3 only (bottom-right) - retreat forward-left
        return RETREAT_FORWARD_LEFT;
      
      // Two sensor detections (edges and sides)
      case 0b0011:  // A0+A1 (both top) - retreat backward
        return RETREAT_BACKWARD;
        
      case 0b1100:  // A2+A3 (both bottom) - retreat forward
        return RETREAT_FORWARD;
        
      case 0b0101:  // A0+A2 (left side) - retreat right
        return RETREAT_RIGHT;
        
      case 0b1010:  // A1+A3 (right side) - retreat left
        return RETREAT_LEFT;
      
      // Diagonal patterns (adjacent corners)
      case 0b1001:  // A0+A3 (diagonal: top-left + bottom-right) - retreat forward-right
        return RETREAT_FORWARD_RIGHT;
        
      case 0b0110:  // A1+A2 (diagonal: top-right + bottom-left) - retreat forward-left
        return RETREAT_FORWARD_LEFT;
      
      // Complex patterns (fallback to nearest safe direction)
      default:
        // If top sensors involved, retreat backward
        if (edges.top_left || edges.top_right) {
          return RETREAT_BACKWARD;
        }
        // If bottom sensors involved, retreat forward
        if (edges.bottom_left || edges.bottom_right) {
          return RETREAT_FORWARD;
        }
        // Shouldn't reach here, but emergency stop as fallback
        return EMERGENCY_STOP;
    }
  }
  
  // ===== PRIORITY 2: Opponent Detection (attack mode) =====
  // Check ToF sensors for opponent
  bool opponent_front = sensors.tof_valid[1] && sensors.tof_distance[1] < 800;  // Front sensor
  bool opponent_left = sensors.tof_valid[0] && sensors.tof_distance[0] < 800;   // Left sensor
  bool opponent_right = sensors.tof_valid[2] && sensors.tof_distance[2] < 800;  // Right sensor
  
  if (opponent_front) {
    return ATTACK_FORWARD;
  }
  
  // ===== PRIORITY 3: Search Mode (default behavior) =====
  return SEARCH_OPPONENT;
}

// Execute action and generate motor commands
MotorCommand executeAction(SumoAction action, const SensorData &sensors) {
  MotorCommand cmd;
  cmd.timestamp = millis();
  cmd.emergency_stop = false;
  
  switch (action) {
    case IDLE:
      cmd.left_speed = 0;
      cmd.right_speed = 0;
      cmd.emergency_stop = true;
      break;
    
    case EMERGENCY_STOP:
      cmd.left_speed = 0;
      cmd.right_speed = 0;
      cmd.emergency_stop = true;
      break;
      
    case SEARCH_OPPONENT:
      // Rotate in place to scan for opponent
      cmd.left_speed = 50;
      cmd.right_speed = 50;
      break;
      
    case ATTACK_FORWARD:
      // Full speed forward attack
      cmd.left_speed = 100;
      cmd.right_speed = -100;
      break;
    
    // ===== Edge Detection Retreat Actions =====
    
    case RETREAT_FORWARD:
      // Both bottom sensors detected - move forward away from rear edge
      cmd.left_speed = 80;
      cmd.right_speed = -80;
      break;
      
    case RETREAT_BACKWARD:
      // Both top sensors detected - move backward away from front edge
      cmd.left_speed = -80;
      cmd.right_speed = 80;
      break;
      
    case RETREAT_FORWARD_LEFT:
      // Bottom-right sensor (A3) or diagonal - retreat forward and turn left
      cmd.left_speed = 40;   // Slower left for turning
      cmd.right_speed = -80;  // Faster right
      break;
      
    case RETREAT_FORWARD_RIGHT:
      // Bottom-left sensor (A2) or diagonal - retreat forward and turn right
      cmd.left_speed = 80;   // Faster left
      cmd.right_speed = -40;  // Slower right for turning
      break;
      
    case RETREAT_BACKWARD_LEFT:
      // Top-right sensor (A1) - retreat backward and turn left
      cmd.left_speed = -40;  // Slower reverse left for turning
      cmd.right_speed = 80; // Faster reverse right
      break;
      
    case RETREAT_BACKWARD_RIGHT:
      // Top-left sensor (A0) - retreat backward and turn right
      cmd.left_speed = -80;  // Faster reverse left
      cmd.right_speed = 40; // Slower reverse right for turning
      break;
      
    case RETREAT_LEFT:
      // Right side sensors (A1+A3) - strafe/turn left aggressively
      cmd.left_speed = -60;  // Reverse left
      cmd.right_speed = -60;  // Forward right (spin left)
      break;
      
    case RETREAT_RIGHT:
      // Left side sensors (A0+A2) - strafe/turn right aggressively
      cmd.left_speed = 60;   // Forward left (spin right)
      cmd.right_speed = 60; // Reverse right
      break;
      
    default:
      // Unknown action - stop as safety measure
      cmd.left_speed = 0;
      cmd.right_speed = 0;
      cmd.emergency_stop = true;
      break;
  }
  
  return cmd;
}