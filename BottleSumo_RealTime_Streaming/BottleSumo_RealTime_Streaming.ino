/*
 * 雙核心 Bottle Sumo Robot - Real-Time Streaming System
 * 使用 Raspberry Pi Pico W 雙核心架構進行高效能控制和即時數據串流
 * 
 * 系統架構:
 * Core 0 (主核心): 馬達控制、戰術邏輯、串口通訊、OLED 顯示、WiFi/TCP 即時串流
 * Core 1 (次核心): 專門負責感測器讀取，提供即時數據 (155.5Hz)
 * 
 * Real-Time Streaming 特色:
 * - 持續串流感測器數據和機器人狀態 (10-20Hz)
 * - JSON 格式數據包含感測器讀數和當前狀態
 * - 非阻塞設計，不影響機器人性能
 * - 支援多客戶端同時連接監控
 * - 自動連線管理和斷線重連
 * 
 * 性能優化:
 * - Core 1: 高速感測器讀取 (155.5Hz)，不受網路影響
 * - Core 0: 邏輯處理 (~100 Hz)、即時串流 (10-20 Hz)
 * - OLED 顯示器: 5Hz 更新，包含 WiFi 狀態和系統資訊
 * - 數據同步: mutex 確保線程安全
 * 
 * 即時串流數據格式:
 * {
 *   "timestamp": 12345,
 *   "sensors": {
 *     "raw": [1234, 5678, 9012, 3456],
 *     "voltage": [1.234, 2.345, 3.456, 0.789]
 *   },
 *   "robot_state": {
 *     "action": "SEARCH_OPPONENT",
 *     "edge_detected": false,
 *     "edge_direction": "SAFE",
 *     "danger_level": 0
 *   },
 *   "system_info": {
 *     "wifi_rssi": -45,
 *     "free_heap": 123456,
 *     "core0_freq": 102.5,
 *     "core1_freq": 847.3
 *   }
 * }
 * 
 * 硬體連接:
 * ADS1115    Raspberry Pi Pico
 * VDD     ←→  3.3V (3V3 OUT)
 * GND     ←→  GND
 * SCL     ←→  SCL (預設引腳，Wire)
 * SDA     ←→  SDA (預設引腳，Wire)
 * 
 * OLED (SSD1306)  Raspberry Pi Pico
 * VDD     ←→  3.3V (3V3 OUT)
 * GND     ←→  GND
 * SCL     ←→  GP27
 * SDA     ←→  GP26
 * 
 * ADS1115    QRE1113 感測器
 * A0      ←→  QRE1113 #1 (前左)
 * A1      ←→  QRE1113 #2 (前右)
 * A2      ←→  QRE1113 #3 (後左)
 * A3      ←→  QRE1113 #4 (後右)
 * 
 * 連接方法:
 * 1. 修改下方的 WiFi 設定
 * 2. 上傳程式到 Raspberry Pi Pico W
 * 3. 連接到機器人 IP，Port 4242
 * 4. 立即開始接收即時數據串流
 * 
 * Python 連接範例:
 * import socket, json
 * s = socket.socket()
 * s.connect(('<機器人IP>', 4242))
 * while True:
 *     data = s.recv(1024)
 *     if data:
 *         try:
 *             json_data = json.loads(data.decode())
 *             print(f"Action: {json_data['robot_state']['action']}")
 *             print(f"Sensors: {json_data['sensors']['voltage']}")
 *         except:
 *             pass
 * 
 * 作者: CTEA-BottleSumo 專案 - 即時串流版本
 * 日期: 2025-09-25
 */

#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>

// ToF Sensor Hardware Configuration
constexpr uint8_t TOF_XSHUT_1 = 12;                   // GP11 - Right sensor shutdown pin
constexpr uint8_t TOF_XSHUT_2 = 11;                   // GP12 - Front sensor shutdown pin
constexpr uint8_t TOF_XSHUT_3 = 13;                   // GP13 - Left sensor shutdown pin
constexpr uint8_t TOF_RIGHT_ADDRESS = 0x30;           // Right sensor I2C address
constexpr uint8_t TOF_FRONT_ADDRESS = 0x31;           // Front sensor I2C address
constexpr uint8_t TOF_LEFT_ADDRESS = 0x32;            // Left sensor I2C address

// ToF Sensor Timing Configuration
constexpr unsigned long TOF_TIMING_BUDGET_US = 20000; // 20ms per reading for ULTRA FAST mode
constexpr uint8_t TOF_VCSEL_PRE_RANGE = 18;           // Pre-range pulse period (speed optimized)
constexpr uint8_t TOF_VCSEL_FINAL_RANGE = 14;         // Final-range pulse period (speed optimized)
constexpr unsigned long TOF_LOOP_DELAY_MS = 70;       // 70ms = 60ms read (3×20ms) + 10ms margin (~14Hz)
constexpr unsigned long TOF_RESET_DELAY_MS = 50;      // Full reset delay
constexpr unsigned long TOF_POST_RESET_DELAY_MS = 20; // Post-reset sensor startup delay (minimal)

// ToF Sensor Detection Configuration
constexpr uint16_t TOF_DETECTION_THRESHOLD_MM = 1600; // 160cm - sumo ring detection distance
constexpr uint16_t TOF_MAX_VALID_RANGE_MM = 1500;     // Maximum sensor range (reduced for better accuracy)
constexpr uint16_t TOF_MIN_VALID_RANGE_MM = 30;       // Minimum sensor range (avoid false positives)
constexpr uint8_t TOF_MAX_VALID_STATUS = 2;           // Accept status 0-2 for reliable readings

// ToF Sensor Initialization Configuration
constexpr uint8_t TOF_MAX_INIT_RETRIES = 3;           // Maximum sensor initialization attempts
constexpr unsigned long TOF_RETRY_DELAY_MS = 100;     // Delay before retry on failure

// ToF Data Freshness Configuration
constexpr unsigned long TOF_DATA_FRESHNESS_MS = 150;  // Data freshness tolerance (Core 1 cycle ~90ms + margin)

namespace Config {
  // Hardware configuration
  constexpr uint8_t OLED_WIDTH = 128;
  constexpr uint8_t OLED_HEIGHT = 64;
  constexpr int8_t OLED_RESET_PIN = -1;
  constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;
  constexpr uint8_t OLED_SDA_PIN = 26;
  constexpr uint8_t OLED_SCL_PIN = 27;

  constexpr uint8_t ADS1115_I2C_ADDRESS = 0x48;
  constexpr uint8_t IR_SENSOR_COUNT = 4;
  constexpr uint8_t TOF_SENSOR_COUNT = 3;
  constexpr uint8_t TOF_INDEX_RIGHT = 0;
  constexpr uint8_t TOF_INDEX_FRONT = 1;
  constexpr uint8_t TOF_INDEX_LEFT = 2;

  // Timing configuration (milliseconds unless noted)
  constexpr unsigned long OLED_UPDATE_INTERVAL_MS = 200;    // 5 Hz
  constexpr unsigned long OLED_START_SCREEN_DURATION_MS = 2000;
  constexpr unsigned long CORE0_LOOP_DELAY_MS = 10;         // 100 Hz
  constexpr unsigned long CORE1_STARTUP_DELAY_MS = 100;
  constexpr unsigned long CORE1_WAIT_POLL_DELAY_MS = 10;
  constexpr unsigned long CORE1_FAILURE_HALT_DELAY_MS = 1000;
  constexpr unsigned long STALE_DATA_DELAY_MS = 5;            // brief pause to let sensors catch up
  constexpr unsigned long STALE_WARNING_THROTTLE_MS = 1000;   // limit stale warning prints (ms)
  constexpr unsigned long DATA_FRESHNESS_TOLERANCE_MS = 100;
  constexpr unsigned long STATUS_PRINT_INTERVAL_MS = 5000;
  constexpr int STATUS_PRINT_LOOP_INTERVAL = 100;
  constexpr int OLED_UPDATE_LOOP_INTERVAL = 50;  // Reduced from 25 to 50 (4Hz -> 2Hz)

  constexpr unsigned long STREAM_SEND_INTERVAL_MS = 50;     // 20 Hz
  constexpr unsigned long TCP_CHECK_INTERVAL_MS = 25;       // 40 Hz
  constexpr unsigned long WIFI_CHECK_INTERVAL_MS = 30000;   // 30 s
  constexpr unsigned long CLIENT_TIMEOUT_MS = 300000;       // 5 min
  constexpr unsigned long WIFI_DISCONNECT_DELAY_MS = 1000;
  constexpr unsigned long MILLISECONDS_PER_SECOND = 1000UL;
  constexpr float MILLISECONDS_PER_SECOND_F = 1000.0f;
  constexpr uint16_t STREAM_SERVER_PORT = 4242;
  constexpr uint32_t I2C_FAST_MODE_HZ = 400000;

  // Thresholds & limits
  constexpr float EDGE_THRESHOLD_VOLTS = 2.5f;
  constexpr int DANGER_LEVEL_EMERGENCY_THRESHOLD = 2;
  constexpr int DANGER_LEVEL_RETREAT_THRESHOLD = 1;
  constexpr int MAX_STREAMING_CLIENTS = 6;
  constexpr unsigned long STREAM_STATS_INTERVAL_MS = 30000;

  // Display heuristics
  constexpr int WIFI_RSSI_STRONG = -50;
  constexpr int WIFI_RSSI_GOOD = -60;
  constexpr int WIFI_RSSI_FAIR = -70;

  constexpr uint8_t I2C_ADDRESS_START = 1;
  constexpr uint8_t I2C_ADDRESS_END = 127;

  // WiFi Access Point configuration
  constexpr const char* AP_SSID = "BottleSumo_Robot";
  constexpr const char* AP_PASSWORD = "sumo2025";
  constexpr uint8_t AP_MAX_CLIENTS = 8;
}

// 創建 OLED 顯示器對象
Adafruit_SSD1306 display(Config::OLED_WIDTH, Config::OLED_HEIGHT, &Wire1, Config::OLED_RESET_PIN);

// OLED 更新控制
unsigned long lastOLEDUpdate = 0;
// ========== Bottle Sumo 控制邏輯 ==========

// Sumo 機器人動作枚舉
enum SumoAction {
  SEARCH_OPPONENT,    // 搜尋對手
  ATTACK_FORWARD,     // 直線攻擊
  RETREAT_AND_TURN,   // 後退並轉向
  EMERGENCY_REVERSE   // 緊急後退
};

// ========== WiFi TCP 即時串流伺服器配置 ==========

// TCP 即時串流伺服器設定
constexpr int TCP_SERVER_PORT = Config::STREAM_SERVER_PORT;           // TCP 伺服器端口
WiFiServer tcpServer(TCP_SERVER_PORT);       // 建立 TCP 伺服器物件

// WiFi 狀態監控 (Access Point)
bool wifiConnected = false;
IPAddress apIpAddress;
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = Config::WIFI_CHECK_INTERVAL_MS;  // 30秒檢查一次 WiFi 狀態

// 即時串流控制
unsigned long lastStreamSend = 0;
const unsigned long STREAM_SEND_INTERVAL = Config::STREAM_SEND_INTERVAL_MS;    // 50ms 發送間隔 (20Hz)
unsigned long lastTCPCheck = 0;
const unsigned long TCP_CHECK_INTERVAL = Config::TCP_CHECK_INTERVAL_MS;      // 25ms TCP檢查間隔 (40Hz)

// 多客戶端即時串流管理
const int MAX_STREAMING_CLIENTS = Config::MAX_STREAMING_CLIENTS;               // 最大同時串流客戶端數
struct StreamingClientState {
  WiFiClient client;                               // WiFi客戶端對象
  unsigned long connectTime;                       // 連接時間
  unsigned long lastStreamTime;                    // 最後串流時間
  unsigned long bytesSent;                         // 已發送字節數
  bool isActive;                                   // 連接狀態
  bool isStreaming;                                // 串流狀態
};

StreamingClientState streamClients[MAX_STREAMING_CLIENTS];  // 串流客戶端連接池
const unsigned long CLIENT_TIMEOUT = Config::CLIENT_TIMEOUT_MS;               // 5分鐘客戶端超時

// ========== 雙核心支援 ==========

// 核心間共享數據結構 (使用 volatile 確保數據同步)
volatile struct SharedSensorData {
  int16_t raw_values[Config::IR_SENSOR_COUNT];      // 感測器原始值
  float voltages[Config::IR_SENSOR_COUNT];          // 感測器電壓值
  bool data_ready;            // 數據準備標誌
  unsigned long timestamp;    // 時間戳
  uint16_t tof_distance[Config::TOF_SENSOR_COUNT];  // ToF 距離 (mm)
  bool tof_valid[Config::TOF_SENSOR_COUNT];         // ToF 讀數是否有效
  uint8_t tof_status[Config::TOF_SENSOR_COUNT];     // ToF 狀態碼
  bool tof_data_ready;                              // ToF 數據準備標誌
  unsigned long tof_timestamp;                      // ToF 時間戳
} shared_data;

// 互斥鎖 (Mutex) 用於數據同步
mutex_t data_mutex;    // Protects shared_data structure
mutex_t wire1_mutex;   // Protects Wire1 I2C bus hardware (OLED + ToF sensors)

// 核心狀態監控
volatile bool core1_active = false;
volatile unsigned long core1_loop_count = 0;
volatile unsigned long core0_loop_count = 0;

// ========== 結構體定義 ==========

// 單一感測器讀數結構體
struct QRE_Reading {
  int16_t raw_value;  // ADC 原始值
  float voltage;      // 轉換後的電壓值
};

// 所有感測器資料結構體
struct QRE_AllSensors {
  QRE_Reading sensor[Config::IR_SENSOR_COUNT];  // 4 個感測器的數據陣列
  
  // 從共享數據更新感測器數據
  void updateFromSharedData() {
    mutex_enter_blocking(&data_mutex);
    if (shared_data.data_ready) {
      for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
        sensor[i].raw_value = shared_data.raw_values[i];
        sensor[i].voltage = shared_data.voltages[i];
      }
    }
    mutex_exit(&data_mutex);
  }
  
  // 檢查數據是否為最新
  bool isDataFresh(unsigned long max_age_ms = Config::DATA_FRESHNESS_TOLERANCE_MS) {
    mutex_enter_blocking(&data_mutex);
    bool fresh = shared_data.data_ready && 
                 (millis() - shared_data.timestamp) < max_age_ms;
    mutex_exit(&data_mutex);
    return fresh;
  }
  
  // 取得特定感測器的原始值
  int16_t getRawValue(int index) {
    if (index >= 0 && index < 4) {
      return sensor[index].raw_value;
    }
    return 0;
  }
  
  // 取得特定感測器的電壓值
  float getVoltage(int index) {
    if (index >= 0 && index < 4) {
      return sensor[index].voltage;
    }
    return 0.0;
  }
  
  // Bottle Sumo 專用：檢測邊緣/白線（高電壓 = 反射 = 白色/邊緣）
  bool isEdgeDetected(float edge_threshold = Config::EDGE_THRESHOLD_VOLTS) {
    for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
      if (sensor[i].voltage > edge_threshold) {
        return true;  // 檢測到邊緣/白線
      }
    }
    return false;
  }
  
  // Bottle Sumo 專用：獲取邊緣方向
  String getEdgeDirection(float edge_threshold = Config::EDGE_THRESHOLD_VOLTS) {
  bool front_left = sensor[0].voltage > edge_threshold;   // 感測器 0: 前左
  bool front_right = sensor[1].voltage > edge_threshold;  // 感測器 1: 前右
  bool back_left = sensor[2].voltage > edge_threshold;    // 感測器 2: 後左
  bool back_right = sensor[3].voltage > edge_threshold;   // 感測器 3: 後右
    
    if (front_left && front_right) return "FRONT";
    if (back_left && back_right) return "BACK";
    if (front_left || back_left) return "LEFT";
    if (front_right || back_right) return "RIGHT";
    if (front_left) return "FRONT_LEFT";
    if (front_right) return "FRONT_RIGHT";
    if (back_left) return "BACK_LEFT";
    if (back_right) return "BACK_RIGHT";
    return "SAFE";
  }
  
  // Bottle Sumo 專用：獲取危險等級（0=安全，4=最危險）
  int getDangerLevel(float edge_threshold = Config::EDGE_THRESHOLD_VOLTS) {
    int danger_count = 0;
    for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
      if (sensor[i].voltage > edge_threshold) {
        danger_count++;
      }
    }
    return danger_count;
  }
};

struct ToFReadings {
  uint16_t distance[Config::TOF_SENSOR_COUNT] = {0};
  bool valid[Config::TOF_SENSOR_COUNT] = {false};
  uint8_t status[Config::TOF_SENSOR_COUNT] = {0};
  bool data_ready = false;
  unsigned long timestamp = 0;

  void updateFromSharedData() {
    mutex_enter_blocking(&data_mutex);
    if (shared_data.tof_data_ready) {
      for (int i = 0; i < Config::TOF_SENSOR_COUNT; ++i) {
        distance[i] = shared_data.tof_distance[i];
        valid[i] = shared_data.tof_valid[i];
        status[i] = shared_data.tof_status[i];
      }
      timestamp = shared_data.tof_timestamp;
      data_ready = true;
    } else {
      for (int i = 0; i < Config::TOF_SENSOR_COUNT; ++i) {
        distance[i] = shared_data.tof_distance[i];
        valid[i] = shared_data.tof_valid[i];
        status[i] = shared_data.tof_status[i];
      }
      timestamp = shared_data.tof_timestamp;
      data_ready = false;
    }
    mutex_exit(&data_mutex);
  }

  bool isDataFresh(unsigned long tolerance_ms = TOF_DATA_FRESHNESS_MS) const {
    return data_ready && (millis() - timestamp) < tolerance_ms;
  }

  uint16_t getDistance(uint8_t index) const {
    if (index < Config::TOF_SENSOR_COUNT) {
      return distance[index];
    }
    return 0;
  }

  String getDirection(uint16_t detection_threshold = TOF_DETECTION_THRESHOLD_MM) const {
    String direction;
    if (valid[Config::TOF_INDEX_FRONT] && distance[Config::TOF_INDEX_FRONT] < detection_threshold) {
      direction += "FRONT ";
    }
    if (valid[Config::TOF_INDEX_RIGHT] && distance[Config::TOF_INDEX_RIGHT] < detection_threshold) {
      direction += "RIGHT ";
    }
    if (valid[Config::TOF_INDEX_LEFT] && distance[Config::TOF_INDEX_LEFT] < detection_threshold) {
      direction += "LEFT ";
    }

    if (direction.length() == 0) {
      return "CLEAR";
    }

    direction.trim();
    return direction;
  }
};

// ========== 全域變數 ==========

Adafruit_ADS1115 ads;  // ADS1115 ADC 物件
Adafruit_VL53L0X lox1, lox2, lox3;  // ToF sensors: lox1=RIGHT, lox2=FRONT, lox3=LEFT
bool tofSensorInitialized[Config::TOF_SENSOR_COUNT] = {false};
volatile bool tofSystemOnline = false;

// Initialize Wire1 I2C bus for OLED and ToF sensors
// MUST be called ONCE before any Wire1 devices are initialized
// Shared by: OLED display (0x3C), ToF sensors (0x30, 0x31, 0x32)
static void initWire1Bus() {
  Wire1.setSDA(Config::OLED_SDA_PIN);
  Wire1.setSCL(Config::OLED_SCL_PIN);
  Wire1.begin();
  Wire1.setClock(Config::I2C_FAST_MODE_HZ);
}

// I2C Scanner for debugging (scans Wire1 bus used by ToF sensors and OLED)
static void scanI2C() {
  Serial.println("========================================");
  Serial.println("I2C Scanner - Wire1 Bus (ToF + OLED)");
  Serial.println("========================================");
  byte error, address;
  int nDevices = 0;
  
  for (address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify known devices
      if (address == 0x29) Serial.print(" (VL53L0X default - active sensor!)");
      else if (address == 0x30) Serial.print(" (ToF RIGHT)");
      else if (address == 0x31) Serial.print(" (ToF FRONT)");
      else if (address == 0x32) Serial.print(" (ToF LEFT)");
      else if (address == 0x3C) Serial.print(" (OLED Display)");
      
      Serial.println();
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (nDevices == 0) {
    Serial.println("⚠️ No I2C devices found!");
    Serial.println("Check wiring and power supply.");
  } else {
    Serial.print("✓ Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)");
  }
  Serial.println("========================================");
}

static String describeOfflineToFSensors() {
  String names;
  // Explicit checks (no loop)
  if (!tofSensorInitialized[Config::TOF_INDEX_RIGHT]) {
    names += "RIGHT";
  }
  if (!tofSensorInitialized[Config::TOF_INDEX_FRONT]) {
    if (names.length() > 0) names += ", ";
    names += "FRONT";
  }
  if (!tofSensorInitialized[Config::TOF_INDEX_LEFT]) {
    if (names.length() > 0) names += ", ";
    names += "LEFT";
  }
  if (names.length() == 0) {
    names = "none";
  }
  return names;
}


// ========== 初始化函數 ==========

// Core 0 (主核心) 初始化 - 負責馬達控制和邏輯
void setup() {
  Serial.begin(115200);
  Serial.println("Bottle Sumo Robot - Real-Time Streaming System");
  Serial.println("Core 0: Motor Control & Real-Time Data Streaming");
  Serial.println("=============================================");
  
  // Initialize mutexes
  mutex_init(&data_mutex);     // For shared data protection
  mutex_init(&wire1_mutex);    // For Wire1 I2C bus protection (OLED + ToF)
  
  // CRITICAL: Initialize Wire1 I2C bus FIRST, before ANY Wire1 devices
  // This bus is shared by OLED (0x3C) and ToF sensors (0x30/0x31/0x32)
  // Must init ONCE to prevent bus reset conflicts between Core 0 and Core 1
  Serial.println("Initializing Wire1 I2C Bus (400kHz Fast Mode)...");
  initWire1Bus();
  Serial.println("✓ Wire1 I2C Bus Ready");
  
  // 初始化共享數據
  shared_data.data_ready = false;
  shared_data.timestamp = 0;
  shared_data.tof_data_ready = false;
  shared_data.tof_timestamp = 0;
  for (int i = 0; i < Config::TOF_SENSOR_COUNT; ++i) {
    shared_data.tof_distance[i] = 0;
    shared_data.tof_valid[i] = false;
    shared_data.tof_status[i] = 0;
    tofSensorInitialized[i] = false;
  }
  tofSystemOnline = false;
  
  // Initialize OLED display
  Serial.println("Initializing OLED Display...");
  if (initOLEDDisplay()) {
    Serial.println("✓ OLED Display Initialization Complete");
    showStartupScreen();  // Show startup screen
  } else {
    Serial.println("⚠️ OLED Display Initialization Failed");
  }
  
  // Wait for Core 1 sensor initialization to complete FIRST
  // This prevents I2C bus conflicts between WiFi and sensor initialization
  Serial.println("Waiting for Core 1 (Sensor Core) Startup...");
  while (!core1_active) {
    delay(Config::CORE1_WAIT_POLL_DELAY_MS);
  }
  Serial.println("✓ Core 1 Sensor Initialization Complete");
  
  // NOW initialize WiFi after sensors are ready
  Serial.println("Initializing WiFi Real-Time Streaming Server...");
  if (initWiFiAndStreamingServer()) {
    Serial.println("✓ WiFi Real-Time Streaming Server Initialization Complete");
  } else {
    Serial.println("⚠️ WiFi Connection Failed, Continue in Offline Mode");
  }
  
  Serial.println("✓ Dual-Core Real-Time Streaming System Initialization Complete");
  printSystemInfo();
}

// Core 1 (次核心) 初始化 - 負責感測器讀取
void setup1() {
  // 等待主核心完成基本初始化
  delay(Config::CORE1_STARTUP_DELAY_MS);
  
  // 初始化感測器系統
  if (initSensorSystem()) {
    core1_active = true;
    // 通過串口通知初始化完成 (小心避免與 Core 0 衝突)
  } else {
    // 初始化失敗，停止執行
    while (1) {
      delay(Config::CORE1_FAILURE_HALT_DELAY_MS);
    }
  }
}

// 初始化感測器系統
bool initSensorSystem() {
  // 初始化 I2C（使用預設引腳）
  Wire.begin();
  Wire.setClock(Config::I2C_FAST_MODE_HZ);  // 設定 I2C 為快速模式 400kHz
  
  // 設定 ADS1115
  ads.setGain(GAIN_ONE);  // ±4.096V 範圍
  ads.setDataRate(RATE_ADS1115_860SPS);  // 最高取樣率 860 SPS
  
  // 初始化 ADS1115（使用設定的 I2C 地址和 Wire）
  if (!ads.begin(Config::ADS1115_I2C_ADDRESS, &Wire)) {
    return false;
  }

  tofSystemOnline = initToFSensors();
  if (!tofSystemOnline) {
    Serial.println(F("⚠️ ToF sensors unavailable; continuing without distance data."));
  }
  
  return true;
}

bool initToFSensors() {
  Serial.println("========================================");
  Serial.println("ToF Sensor Initialization (ULTRA FAST)");
  Serial.println("========================================");
  
  // NOTE: Wire1 already initialized by Core 0 in setup()
  // Do NOT call initWire1Bus() here - it would reset the I2C bus!
  
  // Debug: Scan I2C bus before initialization
  scanI2C();
  
  for (int attempt = 1; attempt <= TOF_MAX_INIT_RETRIES; attempt++) {
    Serial.print("Initialization attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(TOF_MAX_INIT_RETRIES);
    
    // Reset all sensors
    pinMode(TOF_XSHUT_1, OUTPUT);
    pinMode(TOF_XSHUT_2, OUTPUT);
    pinMode(TOF_XSHUT_3, OUTPUT);
    digitalWrite(TOF_XSHUT_1, LOW);
    digitalWrite(TOF_XSHUT_2, LOW);
    digitalWrite(TOF_XSHUT_3, LOW);
    delay(TOF_RESET_DELAY_MS);
    
    bool all_sensors_ok = true;
    
    // Right sensor - ULTRA FAST timing
    digitalWrite(TOF_XSHUT_1, HIGH);
    delay(TOF_POST_RESET_DELAY_MS);
    if (!lox1.begin(TOF_RIGHT_ADDRESS, false, &Wire1)) {
      Serial.println("Failed to init RIGHT sensor");
      tofSensorInitialized[Config::TOF_INDEX_RIGHT] = false;
      all_sensors_ok = false;
      delay(TOF_RETRY_DELAY_MS);
      continue;
    }
    
    // ULTRA FAST: 20ms timing budget
    lox1.setMeasurementTimingBudgetMicroSeconds(TOF_TIMING_BUDGET_US);
    lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, TOF_VCSEL_PRE_RANGE);
    lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, TOF_VCSEL_FINAL_RANGE);
    tofSensorInitialized[Config::TOF_INDEX_RIGHT] = true;
    Serial.println("Right sensor: ULTRA FAST mode (20ms)");

    // Front sensor
    digitalWrite(TOF_XSHUT_2, HIGH);
    delay(TOF_POST_RESET_DELAY_MS);
    if (!lox2.begin(TOF_FRONT_ADDRESS, false, &Wire1)) {
      Serial.println("Failed to init FRONT sensor");
      tofSensorInitialized[Config::TOF_INDEX_FRONT] = false;
      all_sensors_ok = false;
      delay(TOF_RETRY_DELAY_MS);
      continue;
    }
    
    lox2.setMeasurementTimingBudgetMicroSeconds(TOF_TIMING_BUDGET_US);
    lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, TOF_VCSEL_PRE_RANGE);
    lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, TOF_VCSEL_FINAL_RANGE);
    tofSensorInitialized[Config::TOF_INDEX_FRONT] = true;
    Serial.println("Front sensor: ULTRA FAST mode (20ms)");

    // Left sensor
    digitalWrite(TOF_XSHUT_3, HIGH);
    delay(TOF_POST_RESET_DELAY_MS);
    if (!lox3.begin(TOF_LEFT_ADDRESS, false, &Wire1)) {
      Serial.println("Failed to init LEFT sensor");
      tofSensorInitialized[Config::TOF_INDEX_LEFT] = false;
      all_sensors_ok = false;
      delay(TOF_RETRY_DELAY_MS);
      continue;
    }
    
    lox3.setMeasurementTimingBudgetMicroSeconds(TOF_TIMING_BUDGET_US);
    lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, TOF_VCSEL_PRE_RANGE);
    lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, TOF_VCSEL_FINAL_RANGE);
    tofSensorInitialized[Config::TOF_INDEX_LEFT] = true;
    Serial.println("Left sensor: ULTRA FAST mode (20ms)");

    if (all_sensors_ok) {
      Serial.println("All sensors initialized successfully!");
      return true;
    }
  }
  
  Serial.println("CRITICAL: Failed to initialize all sensors after multiple retries!");
  Serial.print("Offline sensors: ");
  Serial.println(describeOfflineToFSensors());
  return false;
}

void readToFSensors(ToFReadings &reading) {
  VL53L0X_RangingMeasurementData_t data1, data2, data3;
  bool anySensorActive = false;

  // CRITICAL: Lock Wire1 bus for entire ToF read cycle (60ms)
  // Prevents Core 0 (OLED) from corrupting I2C transactions
  mutex_enter_blocking(&wire1_mutex);

  // RIGHT sensor (lox1)
  if (!tofSensorInitialized[Config::TOF_INDEX_RIGHT]) {
    reading.status[Config::TOF_INDEX_RIGHT] = 0xFF;
    reading.valid[Config::TOF_INDEX_RIGHT] = false;
    reading.distance[Config::TOF_INDEX_RIGHT] = 0;
  } else {
    anySensorActive = true;
    lox1.rangingTest(&data1, false);
    reading.status[Config::TOF_INDEX_RIGHT] = data1.RangeStatus;
    bool valid = (data1.RangeStatus <= TOF_MAX_VALID_STATUS) &&
                 (data1.RangeMilliMeter >= TOF_MIN_VALID_RANGE_MM) &&
                 (data1.RangeMilliMeter < TOF_MAX_VALID_RANGE_MM);
    reading.valid[Config::TOF_INDEX_RIGHT] = valid;
    reading.distance[Config::TOF_INDEX_RIGHT] = valid ? data1.RangeMilliMeter : 0;
  }

  // FRONT sensor (lox2)
  if (!tofSensorInitialized[Config::TOF_INDEX_FRONT]) {
    reading.status[Config::TOF_INDEX_FRONT] = 0xFF;
    reading.valid[Config::TOF_INDEX_FRONT] = false;
    reading.distance[Config::TOF_INDEX_FRONT] = 0;
  } else {
    anySensorActive = true;
    lox2.rangingTest(&data2, false);
    reading.status[Config::TOF_INDEX_FRONT] = data2.RangeStatus;
    bool valid = (data2.RangeStatus <= TOF_MAX_VALID_STATUS) &&
                 (data2.RangeMilliMeter >= TOF_MIN_VALID_RANGE_MM) &&
                 (data2.RangeMilliMeter < TOF_MAX_VALID_RANGE_MM);
    reading.valid[Config::TOF_INDEX_FRONT] = valid;
    reading.distance[Config::TOF_INDEX_FRONT] = valid ? data2.RangeMilliMeter : 0;
  }

  // LEFT sensor (lox3)
  if (!tofSensorInitialized[Config::TOF_INDEX_LEFT]) {
    reading.status[Config::TOF_INDEX_LEFT] = 0xFF;
    reading.valid[Config::TOF_INDEX_LEFT] = false;
    reading.distance[Config::TOF_INDEX_LEFT] = 0;
  } else {
    anySensorActive = true;
    lox3.rangingTest(&data3, false);
    reading.status[Config::TOF_INDEX_LEFT] = data3.RangeStatus;
    bool valid = (data3.RangeStatus <= TOF_MAX_VALID_STATUS) &&
                 (data3.RangeMilliMeter >= TOF_MIN_VALID_RANGE_MM) &&
                 (data3.RangeMilliMeter < TOF_MAX_VALID_RANGE_MM);
    reading.valid[Config::TOF_INDEX_LEFT] = valid;
    reading.distance[Config::TOF_INDEX_LEFT] = valid ? data3.RangeMilliMeter : 0;
  }

  reading.timestamp = millis();
  reading.data_ready = anySensorActive;
  
  // Release Wire1 bus lock
  mutex_exit(&wire1_mutex);
}

// 初始化 OLED 顯示器
bool initOLEDDisplay() {
  // NOTE: Wire1 already initialized in setup() by initWire1Bus()
  // 初始化 SSD1306 顯示器
  if (!display.begin(SSD1306_SWITCHCAPVCC, Config::OLED_I2C_ADDRESS, &Wire1)) {
    return false;
  }
  
  // 清除顯示緩衝區
  display.clearDisplay();
  display.display();
  
  return true;
}

// Initialize WiFi Access Point and Real-Time Streaming TCP server
bool initWiFiAndStreamingServer() {
  Serial.println("=== WiFi Access Point Real-Time Streaming Server Startup ===");
  Serial.printf("AP SSID: '%s'\n", Config::AP_SSID);
  Serial.printf("AP Password: %s\n", Config::AP_PASSWORD);
  Serial.printf("TCP Streaming Port: %d\n", TCP_SERVER_PORT);
  Serial.printf("Streaming Rate: %luHz (Every %lums)\n",
                Config::MILLISECONDS_PER_SECOND / STREAM_SEND_INTERVAL,
                STREAM_SEND_INTERVAL);

  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(Config::AP_SSID, Config::AP_PASSWORD, 1, 0, Config::AP_MAX_CLIENTS);

  if (!apStarted) {
    Serial.println("❌ Failed to start WiFi Access Point");
    wifiConnected = false;
    return false;
  }

  apIpAddress = WiFi.softAPIP();
  Serial.printf("� Access Point Ready | IP: %s\n", apIpAddress.toString().c_str());
  Serial.println("Connect to the AP and use TCP client to receive streaming data.");

  tcpServer.begin();
  Serial.printf("🔄 Streaming Format: JSON @ %luHz\n", Config::MILLISECONDS_PER_SECOND / STREAM_SEND_INTERVAL);

  for (int i = 0; i < MAX_STREAMING_CLIENTS; i++) {
    streamClients[i].isActive = false;
    streamClients[i].isStreaming = false;
    streamClients[i].bytesSent = 0;
  }

  wifiConnected = true;
  return true;
}

// Display system information
void printSystemInfo() {
  Serial.println("Real-Time Streaming System Specifications:");
  Serial.println("- Processor: Dual-Core RP2040");
  Serial.println("- Core 0: Motor Control & Real-Time Data Streaming");
  Serial.println("- Core 1: High-Speed Sensor Reading (~155.5Hz)");
  Serial.println("- ADC: ADS1115 16-bit");
  Serial.println("- I2C: 400kHz Fast Mode (Dual Channel)");
  Serial.printf("- Sensors: %ux QRE1113\n", Config::IR_SENSOR_COUNT);
  Serial.printf("- Display: SSD1306 %dx%d OLED\n", Config::OLED_WIDTH, Config::OLED_HEIGHT);
  if (wifiConnected) {
    Serial.printf("- WiFi AP: %s (IP: %s)\n", Config::AP_SSID, apIpAddress.toString().c_str());
    Serial.printf("- Real-Time Streaming: Port %d @ %luHz\n", TCP_SERVER_PORT, Config::MILLISECONDS_PER_SECOND / STREAM_SEND_INTERVAL);
    Serial.printf("- Max Concurrent Clients: %d\n", MAX_STREAMING_CLIENTS);
  } else {
    Serial.println("- WiFi: Offline Mode");
  }
  Serial.println("- Real-Time Data: Sensors + Robot State (JSON Format)");
  Serial.println("- Performance: Non-Blocking, Mutex Protected");
  Serial.println("===============================================");
}

// ========== WiFi 和 TCP 即時串流處理函數 ==========

// 處理 WiFi 連接狀態和 TCP 即時串流（優化版 - 最小化 Core 0 負載）
void handleWiFiAndRealTimeStreaming(QRE_AllSensors &sensors, ToFReadings &tofReadings) {
  static unsigned long lastProcessTime = 0;
  unsigned long currentTime = millis();
  
  // 限制 WiFi/TCP 處理頻率，避免影響主迴圈性能
  // 每 25ms 處理一次 WiFi/TCP（40Hz），為主迴圈保留更多時間
  if (currentTime - lastProcessTime < TCP_CHECK_INTERVAL) {
    return; // 提早返回，節省 CPU 時間
  }
  lastProcessTime = currentTime;
  
  // 定期檢查 WiFi 連接狀態（降頻處理）
  if (currentTime - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    checkWiFiConnection();
    lastWiFiCheck = currentTime;
  }
  
  // 如果 WiFi 已連接，處理 TCP 即時串流
  if (wifiConnected) {
    handleStreamingClients(sensors);
    
    // 定期發送即時數據串流
    if (currentTime - lastStreamSend >= STREAM_SEND_INTERVAL) {
      sendRealTimeStreamToAllClients(sensors, tofReadings);
      lastStreamSend = currentTime;
    }
  }
}

// 檢查 WiFi 連接狀態（增強版診斷功能）
void checkWiFiConnection() {
  static uint8_t lastStationCount = 0;

  if (!wifiConnected) {
    Serial.println("⚠️ Access Point inactive. Attempting restart...");
    if (initWiFiAndStreamingServer()) {
      Serial.println("✅ Access Point restarted successfully");
    } else {
      Serial.println("❌ Access Point restart failed");
    }
    return;
  }

  uint8_t currentStations = WiFi.softAPgetStationNum();
  if (currentStations != lastStationCount) {
    Serial.printf("� AP Clients: %d connected\n", currentStations);
    lastStationCount = currentStations;
  }
}

// 處理 TCP 串流客戶端連接管理
void handleStreamingClients(QRE_AllSensors &sensors) {
  // 第1步: 檢查新的客戶端連接 (完全非阻塞)
  WiFiClient newClient = tcpServer.accept();
  if (newClient) {
    // 尋找空閒的客戶端插槽
    int freeSlot = -1;
    for (int i = 0; i < MAX_STREAMING_CLIENTS; i++) {
      if (!streamClients[i].isActive) {
        freeSlot = i;
        break;
      }
    }
    
    if (freeSlot != -1) {
      // 初始化新的串流客戶端
      streamClients[freeSlot].client = newClient;
      streamClients[freeSlot].connectTime = millis();
      streamClients[freeSlot].lastStreamTime = millis();
      streamClients[freeSlot].bytesSent = 0;
      streamClients[freeSlot].isActive = true;
      streamClients[freeSlot].isStreaming = true;
      
      Serial.printf("🔗 新即時串流客戶端連接 [插槽%d]: %s\n", freeSlot, newClient.remoteIP().toString().c_str());
      
      // 發送歡迎訊息和初始化信息
      String welcomeMsg = "{\"message\":\"Bottle Sumo Real-Time Streaming Started\",";
      welcomeMsg += "\"stream_rate_hz\":" + String(1000/STREAM_SEND_INTERVAL) + ",";
      welcomeMsg += "\"data_format\":\"JSON\",";
      welcomeMsg += "\"timestamp\":" + String(millis()) + "}\n";
      
      newClient.print(welcomeMsg);
      streamClients[freeSlot].bytesSent += welcomeMsg.length();
      
    } else {
      // 服務器滿載，拒絕連接
      String rejectMsg = "{\"error\":\"Server full\",\"max_clients\":" + String(MAX_STREAMING_CLIENTS) + "}\n";
      newClient.print(rejectMsg);
      newClient.stop();
      Serial.println("❌ 串流服務器滿載，拒絕新連接");
    }
  }
  
  // 第2步: 檢查並清理斷開的客戶端
  for (int i = 0; i < MAX_STREAMING_CLIENTS; i++) {
    if (!streamClients[i].isActive) continue;
    
    WiFiClient &client = streamClients[i].client;
    
    // 檢查客戶端是否仍然連接
    if (!client.connected()) {
      Serial.printf("🔌 串流客戶端 [插槽%d] 已斷開 (總發送: %lu bytes)\n", i, streamClients[i].bytesSent);
      streamClients[i].isActive = false;
      streamClients[i].isStreaming = false;
      client.stop();
      continue;
    }
    
    // 檢查客戶端超時 (5分鐘無活動)
    if (millis() - streamClients[i].lastStreamTime > CLIENT_TIMEOUT) {
      Serial.printf("⏰ 串流客戶端 [插槽%d] 超時斷開\n", i);
      client.println("{\"message\":\"Connection timeout. Stream ended.\"}");
      client.stop();
      streamClients[i].isActive = false;
      streamClients[i].isStreaming = false;
      continue;
    }
  }
}

String buildIrSensorStreamPayload(const QRE_AllSensors &sensors) {
  String payload = "\"irsensors\":{";
  payload += "\"raw\":[" + String(sensors.sensor[0].raw_value) + "," +
             String(sensors.sensor[1].raw_value) + "," +
             String(sensors.sensor[2].raw_value) + "," +
             String(sensors.sensor[3].raw_value) + "],";
  payload += "\"voltage\":[" + String(sensors.sensor[0].voltage, 3) + "," +
             String(sensors.sensor[1].voltage, 3) + "," +
             String(sensors.sensor[2].voltage, 3) + "," +
             String(sensors.sensor[3].voltage, 3) + "]";
  payload += "}";
  return payload;
}

String buildTofSensorStreamPayload(const ToFReadings &tofReadings) {
  String payload = "\"tof\":{";
  payload += "\"distance_mm\":[" +
             String(tofReadings.distance[Config::TOF_INDEX_RIGHT]) + "," +
             String(tofReadings.distance[Config::TOF_INDEX_FRONT]) + "," +
             String(tofReadings.distance[Config::TOF_INDEX_LEFT]) + "],";
  payload += "\"valid\":[";
  payload += (tofReadings.valid[Config::TOF_INDEX_RIGHT] ? "true" : "false");
  payload += ",";
  payload += (tofReadings.valid[Config::TOF_INDEX_FRONT] ? "true" : "false");
  payload += ",";
  payload += (tofReadings.valid[Config::TOF_INDEX_LEFT] ? "true" : "false");
  payload += "],";
  payload += "\"status\":[" +
             String(tofReadings.status[Config::TOF_INDEX_RIGHT]) + "," +
             String(tofReadings.status[Config::TOF_INDEX_FRONT]) + "," +
             String(tofReadings.status[Config::TOF_INDEX_LEFT]) + "],";
  payload += "\"direction\":\"" + tofReadings.getDirection() + "\"";
  payload += "}";
  return payload;
}

// 發送即時數據串流到所有連接的客戶端
void sendRealTimeStreamToAllClients(QRE_AllSensors &sensors, ToFReadings &tofReadings) {
  // 計算系統性能指標
  float core0_freq = core0_loop_count * 1000.0 / millis();
  float core1_freq = core1_loop_count * 1000.0 / millis();

  // 決定機器人行動狀態
  SumoAction currentAction = decideSumoAction(sensors);
  String actionString = getActionString(currentAction);

  // 構建即時串流 JSON 數據包
  String streamData = "{";
  streamData += "\"timestamp\":" + String(millis()) + ",";

  // 感測器數據 - QRE 紅外線陣列
  streamData += buildIrSensorStreamPayload(sensors) + ",";

  // 感測器數據 - ToF 距離陣列
  streamData += buildTofSensorStreamPayload(tofReadings) + ",";

  // 機器人狀態
  streamData += "\"robot_state\":{";
  streamData += "\"action\":\"" + actionString + "\",";
  streamData += "\"edge_detected\":" + String(sensors.isEdgeDetected() ? "true" : "false") + ",";
  streamData += "\"edge_direction\":\"" + sensors.getEdgeDirection() + "\",";
  streamData += "\"danger_level\":" + String(sensors.getDangerLevel());
  streamData += "},";

  // 系統資訊
  streamData += "\"system_info\":{";
  streamData += "\"free_heap\":" + String(rp2040.getFreeHeap()) + ",";
  streamData += "\"core0_freq\":" + String(core0_freq, 1) + ",";
  streamData += "\"core1_freq\":" + String(core1_freq, 1) + ",";
  streamData += "\"active_clients\":" + String(getActiveClientCount()) + ",";
  streamData += "\"ap_clients\":" + String(WiFi.softAPgetStationNum());
  streamData += "}";

  streamData += "}\n";

  // 發送到所有活躍的串流客戶端
  int sentCount = 0;
  for (int i = 0; i < MAX_STREAMING_CLIENTS; i++) {
    if (streamClients[i].isActive && streamClients[i].isStreaming) {
      WiFiClient &client = streamClients[i].client;

      // 檢查客戶端寫入緩衝區是否可用
      if (client.availableForWrite() > streamData.length()) {
        client.print(streamData);
        streamClients[i].bytesSent += streamData.length();
        streamClients[i].lastStreamTime = millis();
        sentCount++;
      } else {
        // 緩衝區滿，跳過這次發送避免阻塞
        Serial.printf("⚠️ 客戶端[%d] 緩衝區滿，跳過此次串流\n", i);
      }
    }
  }

  // 每30秒輸出一次串流統計
  static unsigned long lastStats = 0;
  if (millis() - lastStats > Config::STREAM_STATS_INTERVAL_MS) {
    Serial.printf("📊 串流統計: %d 活躍客戶端, 平均 %dHz, 封包大小 %d bytes\n",
                  sentCount, 1000/STREAM_SEND_INTERVAL, streamData.length());
    lastStats = millis();
  }
}

// 獲取動作字符串
String getActionString(SumoAction action) {
  switch(action) {
    case SEARCH_OPPONENT:
      return "SEARCH_OPPONENT";
    case ATTACK_FORWARD:
      return "ATTACK_FORWARD";
    case RETREAT_AND_TURN:
      return "RETREAT_AND_TURN";
    case EMERGENCY_REVERSE:
      return "EMERGENCY_REVERSE";
    default:
      return "UNKNOWN";
  }
}

// 獲取活躍客戶端數量
int getActiveClientCount() {
  int count = 0;
  for (int i = 0; i < MAX_STREAMING_CLIENTS; i++) {
    if (streamClients[i].isActive && streamClients[i].isStreaming) {
      count++;
    }
  }
  return count;
}

// ========== OLED 顯示函數 ==========

// 顯示啟動畫面
void showStartupScreen() {
  display.clearDisplay();
  
  // 設定文字顏色和大小
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // 標題
  display.setCursor(8, 0);
  display.println("BOTTLE SUMO ROBOT");
  
  // 分隔線
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // 系統信息
  display.setCursor(0, 15);
  display.println("Real-Time Streaming");
  display.setCursor(0, 25);
  display.println("Core 0: Streaming");
  display.setCursor(0, 35);
  display.println("Core 1: Sensors 155.5Hz");
  
  // 硬體信息
  display.setCursor(0, 45);
  display.println("JSON @ 20Hz Stream");
  
  // 狀態指示
  display.setCursor(85, 55);
  display.println("READY!");
  
  display.display();
  delay(2000);  // 顯示 2 秒
}

// 更新 OLED 主顯示（包含串流狀態）
void updateOLEDDisplay(QRE_AllSensors &sensors, ToFReadings &tofReadings) {
  // Clear OLED buffer before drawing the next frame to avoid ghosting
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextWrap(false);
  
  // 標題行
  // WiFi 狀態行
  display.setCursor(0, 0);
  if (wifiConnected) {
    display.print("AP: ");
    display.print(apIpAddress.toString());
    display.printf(" C:%d", WiFi.softAPgetStationNum());
  } else {
    display.print("WiFi: OFFLINE");
  }
  
  // 串流狀態
  display.setCursor(0, 15);
  if (wifiConnected) {
    int activeClients = getActiveClientCount();
    display.printf("Stream: %dHz %dclients", 1000/STREAM_SEND_INTERVAL, activeClients);
  } else {
    display.print("Stream: DISABLED");
  }
  
  // 機器人狀態
  display.setCursor(0, 25);
  SumoAction currentAction = decideSumoAction(sensors);
  String actionString = getActionString(currentAction);
  display.print("Action: ");
  // 縮短顯示文字以適應螢幕
  if (actionString == "SEARCH_OPPONENT") display.print("SEARCH");
  else if (actionString == "RETREAT_AND_TURN") display.print("RETREAT");
  else if (actionString == "EMERGENCY_REVERSE") display.print("EMERGENCY");
  else if (actionString == "ATTACK_FORWARD") display.print("ATTACK");
  else display.print("UNKNOWN");
  
  // 感測器狀態 (以兩行呈現，保持可讀性)
  static const char* SENSOR_LABELS[Config::IR_SENSOR_COUNT] = {"FL", "FR", "RL", "RR"};
  const float edgeThreshold = Config::EDGE_THRESHOLD_VOLTS;
  for (int row = 0; row < 2; ++row) {
    display.setCursor(0, 32 + row * 8);
    for (int col = 0; col < 2; ++col) {
      int index = row * 2 + col;
      display.print(SENSOR_LABELS[index]);
      display.print(":");
      display.print(sensors.sensor[index].voltage > edgeThreshold ? "E" : "S");
      display.print("  ");
    }
  }
  
  // 危險等級
  display.setCursor(90, 32);
  display.printf("D:%d/4", sensors.getDangerLevel());
  
  // ToF 距離摘要 (取整數顯示，避免佔用空間)
  display.setCursor(0, 48);
  auto printToFValue = [&](uint8_t index) {
    if (tofReadings.valid[index]) {
      display.printf("%d", tofReadings.distance[index]);
    } else {
      display.print("--");
    }
  };
  if (tofReadings.data_ready) {
    display.print("ToF R:");
    printToFValue(Config::TOF_INDEX_RIGHT);
    display.print(" F:");
    printToFValue(Config::TOF_INDEX_FRONT);
    display.print(" L:");
    printToFValue(Config::TOF_INDEX_LEFT);
  } else if (!tofSystemOnline) {
    display.print("ToF: OFFLINE");
  } else {
    display.print("ToF: -- -- --");
  }
  
  // 系統狀態行
  display.setCursor(0, 56);
  display.printf("Uptime:%lus", millis()/1000);
  
  // Core 頻率狀態
  display.setCursor(70, 56);
  display.printf("C0:%.1fHz C1:%.1fHz", 
                 core0_loop_count * 1000.0 / millis(),
                 core1_loop_count * 1000.0 / millis());
  
  // CRITICAL: Lock Wire1 bus before display update (~20ms I2C transaction)
  // Prevents Core 1 (ToF) from corrupting OLED communication
  mutex_enter_blocking(&wire1_mutex);
  display.display();  // Send 1024 bytes over I2C (20ms blocking)
  mutex_exit(&wire1_mutex);
}

// ========== 感測器讀取函數 ==========

// Core 1 專用：更新共享 IR 感測器數據 (高速)
void updateSharedIRData() {
  static int16_t raw_values[Config::IR_SENSOR_COUNT];
  static float voltages[Config::IR_SENSOR_COUNT];
  
  // 讀取所有 IR 感測器 (快速，約 860 SPS)
  for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
    raw_values[i] = ads.readADC_SingleEnded(i);
    voltages[i] = ads.computeVolts(raw_values[i]);
  }

  unsigned long irTimestamp = millis();
  
  // 更新共享數據 (使用互斥鎖確保數據完整性)
  mutex_enter_blocking(&data_mutex);
  for (int i = 0; i < Config::IR_SENSOR_COUNT; i++) {
    shared_data.raw_values[i] = raw_values[i];
    shared_data.voltages[i] = voltages[i];
  }
  shared_data.timestamp = irTimestamp;
  shared_data.data_ready = true;
  mutex_exit(&data_mutex);
}

// Core 1 專用：更新共享 ToF 感測器數據 (慢速，獨立更新)
void updateSharedToFData() {
  if (!tofSystemOnline) {
    return;
  }

  ToFReadings tofReading;
  readToFSensors(tofReading);

  if (tofReading.data_ready) {
    mutex_enter_blocking(&data_mutex);
    for (int i = 0; i < Config::TOF_SENSOR_COUNT; ++i) {
      shared_data.tof_distance[i] = tofReading.distance[i];
      shared_data.tof_valid[i] = tofReading.valid[i];
      shared_data.tof_status[i] = tofReading.status[i];
    }
    shared_data.tof_timestamp = tofReading.timestamp;
    shared_data.tof_data_ready = true;
    mutex_exit(&data_mutex);
  } else {
    mutex_enter_blocking(&data_mutex);
    shared_data.tof_data_ready = false;
    shared_data.tof_timestamp = tofReading.timestamp;
    for (int i = 0; i < Config::TOF_SENSOR_COUNT; ++i) {
      shared_data.tof_distance[i] = 0;
      shared_data.tof_valid[i] = false;
      shared_data.tof_status[i] = tofSensorInitialized[i] ? 0 : 0xFF;
    }
    mutex_exit(&data_mutex);
  }
}

// Core 0 專用：從共享數據獲取感測器資訊
QRE_AllSensors getAllSensorsFromShared() {
  QRE_AllSensors all_sensors;
  all_sensors.updateFromSharedData();
  return all_sensors;
}

ToFReadings getToFReadingsFromShared() {
  ToFReadings readings;
  readings.updateFromSharedData();
  return readings;
}

// ========== Bottle Sumo 戰術函數 ==========

// 根據感測器狀態決定 Sumo 行動
SumoAction decideSumoAction(QRE_AllSensors &sensors) {
  const float EDGE_THRESHOLD = 2.5;  // 邊緣檢測閾值
  
  int danger_level = sensors.getDangerLevel(EDGE_THRESHOLD);
  
  // 危險等級判斷
  if (danger_level >= 2) {
    return EMERGENCY_REVERSE;  // 多個感測器檢測到邊緣
  } else if (danger_level == 1) {
    return RETREAT_AND_TURN;   // 單個感測器檢測到邊緣
  } else {
    return SEARCH_OPPONENT;    // 沒有檢測到邊緣，繼續搜尋
  }
}

// 執行 Sumo 動作（這裡只顯示建議，實際馬達控制需要您實現）
void executeSumoAction(SumoAction action, QRE_AllSensors &sensors) {
  switch (action) {
    case SEARCH_OPPONENT:
      // Serial.println("🔍 搜尋對手 - 建議: 前進/旋轉尋找目標");
      // TODO: 實現搜尋邏輯
      // motorForward(speed_low);
      // 或 motorRotate(direction);
      break;
      
    case ATTACK_FORWARD:
      // Serial.println("⚔️ 攻擊模式 - 建議: 全速前進");
      // TODO: 實現攻擊邏輯
      // motorForward(speed_max);
      break;
      
    case RETREAT_AND_TURN: {
      // Serial.println("↩️ 後退轉向 - 建議: 避開邊緣並重新定位");
      // TODO: 實現後退轉向邏輯
      String direction = sensors.getEdgeDirection();
      // Serial.print("   危險方向: ");
      // Serial.println(direction);
      // motorBackward(speed_medium);
      // delay(500);
      // motorTurn(opposite_direction);
      break;
    }
      
    case EMERGENCY_REVERSE:
      // Serial.println("🚨 緊急後退 - 建議: 立即遠離邊緣");
      // TODO: 實現緊急後退邏輯
      // motorBackward(speed_max);
      break;
  }
}

// Real-Time Streaming 狀態顯示（簡化版，避免串口輸出過多）
void printStreamingStatus(QRE_AllSensors &sensors, ToFReadings &tofReadings) {
  static unsigned long lastPrint = 0;
  
  // 每5秒輸出一次狀態（避免串口被淹沒）
  if (millis() - lastPrint < 5000) return;
  lastPrint = millis();
  
  Serial.println("========== Real-Time Streaming Status ==========");
  
  // Core 狀態監控
  Serial.printf("Core 0: %.1fHz | Core 1: %.1fHz\n", 
               core0_loop_count * 1000.0 / millis(),
               core1_loop_count * 1000.0 / millis());
  
  // 數據新鮮度檢查
  Serial.printf("Data Status: %s\n", sensors.isDataFresh() ? "✅ Fresh" : "⚠️ Stale");
  
  // 機器人狀態
  SumoAction action = decideSumoAction(sensors);
  Serial.printf("Robot Action: %s\n", getActionString(action).c_str());
  Serial.printf("Edge Detection: %s\n", sensors.isEdgeDetected() ? "⚠️ Detected!" : "✅ Safe");
  Serial.printf("Danger Level: %d/4\n", sensors.getDangerLevel());

  if (!tofSystemOnline) {
    Serial.println("ToF System: OFFLINE (no sensors detected)");
  } else if (tofReadings.data_ready) {
    String rightStr = tofReadings.valid[Config::TOF_INDEX_RIGHT]
                        ? String(tofReadings.distance[Config::TOF_INDEX_RIGHT])
                        : String("--");
    String frontStr = tofReadings.valid[Config::TOF_INDEX_FRONT]
                        ? String(tofReadings.distance[Config::TOF_INDEX_FRONT])
                        : String("--");
    String leftStr = tofReadings.valid[Config::TOF_INDEX_LEFT]
                        ? String(tofReadings.distance[Config::TOF_INDEX_LEFT])
                        : String("--");
    Serial.printf("ToF Distances (mm): R=%s F=%s L=%s | Direction: %s | Fresh: %s\n",
                  rightStr.c_str(),
                  frontStr.c_str(),
                  leftStr.c_str(),
                  tofReadings.getDirection().c_str(),
                  tofReadings.isDataFresh() ? "✅" : "⚠️");
  } else {
    Serial.println("ToF Data: Awaiting first measurement");
  }
  
  // 串流狀態
  if (wifiConnected) {
    int activeClients = getActiveClientCount();
    Serial.printf("Streaming: %d clients @ %dHz | AP Clients: %d\n", activeClients, 1000/STREAM_SEND_INTERVAL, WiFi.softAPgetStationNum());
  } else {
    Serial.println("Streaming: OFFLINE");
  }
  
  Serial.println("===============================================");
}

// ========== 主循環 ==========

// Core 0 主循環 - 馬達控制與即時串流
void loop() {
  static int loop_count = 0;
  static bool irDataPrimed = false;
  static bool tofDataPrimed = false;
  static unsigned long lastIrStaleWarningMs = 0;
  static unsigned long lastTofStaleWarningMs = 0;
  
  // 更新循環計數器
  core0_loop_count++;
  
  // 從 Core 1 獲取最新感測器數據
  QRE_AllSensors all_sensors = getAllSensorsFromShared();
  ToFReadings tofReadings = getToFReadingsFromShared();
  
  // 檢查數據新鮮度
  bool irFresh = all_sensors.isDataFresh(Config::DATA_FRESHNESS_TOLERANCE_MS);
  if (irFresh) {
    irDataPrimed = true;
  } else if (!irDataPrimed) {
    delay(Config::STALE_DATA_DELAY_MS);
    return;
  } else {
    unsigned long now = millis();
    if (now - lastIrStaleWarningMs >= Config::STALE_WARNING_THROTTLE_MS) {
      Serial.println(F("⚠️ 警告: 感測器數據過時!"));
      lastIrStaleWarningMs = now;
    }
    delay(Config::STALE_DATA_DELAY_MS);
    return;
  }

  bool tofFresh = tofReadings.isDataFresh();
  if (tofFresh) {
    tofDataPrimed = true;
  } else if (tofReadings.data_ready) {
    unsigned long now = millis();
    if (tofDataPrimed && (now - lastTofStaleWarningMs) >= Config::STALE_WARNING_THROTTLE_MS) {
      Serial.println(F("⚠️ 警告: ToF 感測器數據過時!"));
      lastTofStaleWarningMs = now;
    }
  }
  
  // 每 100 次循環顯示狀態（降低頻率避免串口堵塞）
  if (loop_count % 100 == 0) {
    printStreamingStatus(all_sensors, tofReadings);
    
    // 決定並執行動作
    SumoAction action = decideSumoAction(all_sensors);
    executeSumoAction(action, all_sensors);
  }
  
  // 每 50 次循環更新 OLED 顯示（減少 Wire1 阻塞，從 4Hz → 2Hz）
  // OLED display.display() blocks Wire1 for ~20ms, reduce frequency to help Core 1
  if (loop_count % 50 == 0) {
    updateOLEDDisplay(all_sensors, tofReadings);
  }
  
  loop_count++;
  
  // WiFi 狀態監控和 TCP 即時串流處理
  handleWiFiAndRealTimeStreaming(all_sensors, tofReadings);
  
  // 馬達控制邏輯在這裡執行
  // TODO: 添加您的馬達控制代碼
  
  // 適當的延遲，讓 Core 0 以適合馬達控制的頻率運行 (例如 100Hz)
  delay(10);  // 100Hz
}

// Core 1 主循環 - 專門負責感測器讀取
void loop1() {
  static unsigned long lastToFUpdate = 0;
  
  // 更新循環計數器
  core1_loop_count++;
  
  // 持續高速讀取 IR 感測器並更新共享數據
  // IR sensors are fast (~860 SPS), so we read them continuously
  updateSharedIRData();
  
  // ToF sensors are slower (100ms timing budget), update at ~10Hz
  unsigned long now = millis();
  if ((now - lastToFUpdate) >= TOF_LOOP_DELAY_MS) {
    updateSharedToFData();
    lastToFUpdate = now;
  }
  
  // 最小延遲，讓 Core 1 以最高速度運行 IR 讀取
  // 實際速度由 ADS1115 的讀取速度決定 (~860 SPS)
}

// ========== 使用說明和範例 ==========

/*
 * Real-Time Streaming 使用方法:
 * 
 * 1. Python 連接範例:
 *    import socket, json, time
 *    
 *    s = socket.socket()
 *    s.connect(('<機器人IP>', 4242))
 *    
 *    while True:
 *        try:
 *            data = s.recv(1024)
 *            if data:
 *                json_data = json.loads(data.decode())
 *                print(f"時間: {json_data['timestamp']}")
 *                print(f"動作: {json_data['robot_state']['action']}")
 *                print(f"感測器: {json_data['sensors']['voltage']}")
 *                print(f"邊緣檢測: {json_data['robot_state']['edge_detected']}")
 *                print("---")
 *        except Exception as e:
 *            print(f"錯誤: {e}")
 *            break
 * 
 * 2. JavaScript/Node.js 連接範例:
 *    const net = require('net');
 *    
 *    const client = new net.Socket();
 *    client.connect(4242, '<機器人IP>', () => {
 *        console.log('Connected to Bottle Sumo Robot');
 *    });
 *    
 *    client.on('data', (data) => {
 *        try {
 *            const jsonData = JSON.parse(data.toString());
 *            console.log('Action:', jsonData.robot_state.action);
 *            console.log('Sensors:', jsonData.sensors.voltage);
 *        } catch (e) {
 *            // JSON parsing error, ignore
 *        }
 *    });
 * 
 * 3. 終端機連接測試:
 *    telnet <機器人IP> 4242
 *    或
 *    nc <機器人IP> 4242
 * 
 * 特色:
 * - 連接後立即開始接收即時數據串流
 * - JSON 格式包含完整的感測器和機器人狀態信息
 * - 20Hz 更新頻率，適合即時監控和分析
 * - 支援最多 6 個客戶端同時連接
 * - 非阻塞設計，不影響機器人性能
 * - 自動斷線重連和錯誤處理
 * - 包含系統性能監控資訊
 */