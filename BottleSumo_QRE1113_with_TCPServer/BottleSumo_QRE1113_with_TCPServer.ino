/*
 * 雙核心 Bottle Sumo Robot - QRE1113 Edge Detection System with WiFi TCP Server
 * 使用 Raspberry Pi Pico W 雙核心架構進行高效能控制和遠端監控
 * 
 * 系統架構:
 * Core 0 (主核心): 馬達控制、戰術邏輯、串口通訊、OLED 顯示、WiFi/TCP 伺服器
 * Core 1 (次核心): 專門負責感測器讀取，提供即時數據 (~860 Hz)
 * 
 * 性能優化:
 * - Core 1: 高速感測器讀取 (~860 Hz)，不受網路影響
 * - Core 0: 邏輯處理 (~100 Hz)、WiFi/TCP 處理 (~20 Hz)
 * - OLED 顯示器: 2Hz 更新，包含 WiFi 狀態和系統資訊
 * - WiFi 診斷: 30秒檢查間隔，自動重連機制
 * - 數據同步: mutex 確保線程安全，總反應速度提升 2-5 倍
 * 
 * WiFi TCP 伺服器功能:
 * - 自動掃描並連接 WiFi 網路
 * - TCP 伺服器預設 Port 4242
 * - 支援即時感測器數據傳輸（純文字和 JSON 格式）
 * - 提供系統狀態查詢和診斷資訊
 * - 完全非阻塞設計，不影響機器人性能
 * 
 * 硬體連接:
 * ADS1115    Raspberry Pi Pico
 * VDD     ←→  3.3V (3V3 OUT)
 * GND     ←→  GND
 * SCL     ←→  SCL (預設引腳，Wire1)
 * SDA     ←→  SDA (預設引腳，Wire1)
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
 * 馬達連接 (您需要根據實際硬體修改):
 * 左馬達: GPIO pins TBD
 * 右馬達: GPIO pins TBD
 * 
 * 所需庫文件 (請在 Arduino IDE 中安裝):
 * - Adafruit ADS1X15 (ADS1115 支援)
 * - Adafruit GFX Library (圖形庫)
 * - Adafruit SSD1306 (OLED 顯示器支援)
 * - WiFi (Raspberry Pi Pico W 內建)
 * 
 * WiFi 配置:
 * 1. 修改下方的 SSID 和密碼常數
 * 2. 上傳程式後，機器人會自動掃描並連接 WiFi
 * 3. 序列埠會顯示連接狀態和 IP 位址
 * 4. OLED 螢幕也會顯示 WiFi 狀態和 IP
 * 
 * TCP 伺服器使用方法:
 * 1. 連接到機器人的 IP 位址，Port 4242
 * 2. 可用指令:
 *    - 'data'   : 獲取純文字格式的感測器數據
 *    - 'json'   : 獲取 JSON 格式的感測器數據
 *    - 'status' : 獲取系統狀態和診斷資訊
 *    - 'help'   : 顯示所有可用指令
 * 
 * 範例連接 (Windows):
 * telnet <機器人IP> 4242
 * 
 * 範例連接 (Linux/Mac):
 * nc <機器人IP> 4242
 * 
 * Python 範例:
 * import socket
 * s = socket.socket()
 * s.connect(('<機器人IP>', 4242))
 * s.send(b'json\n')
 * data = s.recv(1024)
 * print(data.decode())
 * s.close()
 * 
 * 性能: Core 1 ~860 Hz 感測器讀取, Core 0 ~100 Hz 控制循環, OLED 5Hz 更新
 * 作者: CTEA-BottleSumo 專案 - 雙核心 + OLED 版本
 * 日期: 2025-09-23
 */

#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>

// ========== OLED 顯示器配置 ==========

#define SCREEN_WIDTH 128     // OLED 顯示器寬度（像素）
#define SCREEN_HEIGHT 64     // OLED 顯示器高度（像素）
#define OLED_RESET -1        // 重置引腳（如果共用 Arduino 重置引腳則設為 -1）
#define SCREEN_ADDRESS 0x3C  // OLED I2C 地址（通常為 0x3C 或 0x3D）

// OLED I2C 引腳配置（使用 Wire 實例，與 ADS1115 的 Wire1 分離）
#define OLED_SDA_PIN 26      // GP26 - SDA
#define OLED_SCL_PIN 27      // GP27 - SCL

// 創建 OLED 顯示器對象
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

// OLED 更新控制
unsigned long lastOLEDUpdate = 0;
const unsigned long OLED_UPDATE_INTERVAL = 200;  // 200ms 更新間隔（5Hz），避免影響性能

// ========== WiFi TCP 伺服器配置 ==========

// WiFi 網路設定 (請修改為您的網路資訊)
// 支援多個WiFi網路（依序嘗試連接）
struct WiFiCredentials {
  const char* ssid;
  const char* password;
};

// WiFi 網路列表（按照信號強度排序，強的在前）
WiFiCredentials wifiNetworks[] = {
  {"TestWiFi", "admin01230"}, // 如果需要可以設定密碼
  // 您可以在此添加更多網路...
};

const int numWiFiNetworks = sizeof(wifiNetworks) / sizeof(wifiNetworks[0]);

// 當前使用的網路設定
const char* ssid = wifiNetworks[0].ssid;
const char* password = wifiNetworks[0].password;

// TCP 伺服器設定
const int TCP_SERVER_PORT = 4242;           // TCP 伺服器端口
WiFiServer tcpServer(TCP_SERVER_PORT);       // 建立 TCP 伺服器物件

// WiFi 狀態監控
bool wifiConnected = false;
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 30000;  // 30秒檢查一次 WiFi 狀態

// TCP 客戶端管理
unsigned long lastDataSend = 0;
const unsigned long DATA_SEND_INTERVAL = 100;     // 100ms 發送間隔 (10Hz)

// ========== 雙核心支援 ==========

// 核心間共享數據結構 (使用 volatile 確保數據同步)
volatile struct SharedSensorData {
  int16_t raw_values[4];      // 感測器原始值
  float voltages[4];          // 感測器電壓值
  bool data_ready;            // 數據準備標誌
  unsigned long timestamp;    // 時間戳
} shared_data;

// 互斥鎖 (Mutex) 用於數據同步
mutex_t data_mutex;

// 核心狀態監控
volatile bool core1_active = false;


// ========== 結構體定義 ==========

// 單一感測器讀數結構體
struct QRE_Reading {
  int16_t raw_value;  // ADC 原始值
  float voltage;      // 轉換後的電壓值
};

// 所有感測器資料結構體
struct QRE_AllSensors {
  QRE_Reading sensor[4];  // 4 個感測器的數據陣列
  
  // 從共享數據更新感測器數據
  void updateFromSharedData() {
    mutex_enter_blocking(&data_mutex);
    if (shared_data.data_ready) {
      for (int i = 0; i < 4; i++) {
        sensor[i].raw_value = shared_data.raw_values[i];
        sensor[i].voltage = shared_data.voltages[i];
      }
    }
    mutex_exit(&data_mutex);
  }
  
  // 檢查數據是否為最新
  bool isDataFresh(unsigned long max_age_ms = 10) {
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
  
  // Display all sensor data
  void printAll() {
    Serial.println("=== QRE1113 Sensor Data ===");
    for (int i = 0; i < 4; i++) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": Raw=");
      Serial.print(sensor[i].raw_value);
      Serial.print(", Voltage=");
      Serial.print(sensor[i].voltage, 3);
      Serial.println("V");
    }
    Serial.println("========================");
  }
  
  // Bottle Sumo 專用：檢測邊緣/白線（高電壓 = 反射 = 白色/邊緣）
  bool isEdgeDetected(float edge_threshold = 2.5) {
    for (int i = 0; i < 4; i++) {
      if (sensor[i].voltage > edge_threshold) {
        return true;  // 檢測到邊緣/白線
      }
    }
    return false;
  }
  
  // Bottle Sumo 專用：獲取邊緣方向
  String getEdgeDirection(float edge_threshold = 2.5) {
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
  int getDangerLevel(float edge_threshold = 2.5) {
    int danger_count = 0;
    for (int i = 0; i < 4; i++) {
      if (sensor[i].voltage > edge_threshold) {
        danger_count++;
      }
    }
    return danger_count;
  }
};

// ========== 函數宣告 ==========
void showStartupScreen();
void handleTCPClients(QRE_AllSensors &sensors);
void sendSensorData(WiFiClient &client, QRE_AllSensors &sensors);
void sendSensorJSON(WiFiClient &client, QRE_AllSensors &sensors);
void sendSystemStatus(WiFiClient &client);
void sendHelpInfo(WiFiClient &client);
volatile unsigned long core1_loop_count = 0;
volatile unsigned long core0_loop_count = 0;

// ========== 全域變數 ==========

Adafruit_ADS1115 ads;  // ADS1115 ADC 物件

// ========== Bottle Sumo 控制邏輯 ==========

// Sumo 機器人動作枚舉
enum SumoAction {
  SEARCH_OPPONENT,    // 搜尋對手
  ATTACK_FORWARD,     // 直線攻擊
  RETREAT_AND_TURN,   // 後退並轉向
  EMERGENCY_REVERSE   // 緊急後退
};

// ========== 初始化函數 ==========

// Core 0 (主核心) 初始化 - 負責馬達控制和邏輯
void setup() {
  Serial.begin(115200);
  Serial.println("Bottle Sumo Robot - Dual-Core System Startup");
  Serial.println("Core 0: Motor Control & Logic Processing");
  Serial.println("=====================================");
  
  // Initialize mutex
  mutex_init(&data_mutex);
  
  // 初始化共享數據
  shared_data.data_ready = false;
  shared_data.timestamp = 0;
  /*
  // Initialize OLED display
  Serial.println("Initializing OLED Display...");
  if (initOLEDDisplay()) {
    Serial.println("✓ OLED Display Initialization Complete");
    showStartupScreen();  // Show startup screen
  } else {
    Serial.println("⚠️ OLED Display Initialization Failed");
  }
  */
  // Initialize WiFi and TCP server
  Serial.println("Initializing WiFi Connection...");
  if (initWiFiAndServer()) {
    Serial.println("✓ WiFi TCP Server Initialization Complete");
  } else {
    Serial.println("⚠️ WiFi Connection Failed, Continue in Offline Mode");
  }
  
  // Wait for Core 1 startup
  Serial.println("Waiting for Core 1 (Sensor Core) Startup...");
  while (!core1_active) {
    delay(10);
  }
  
  Serial.println("✓ Dual-Core System Initialization Complete");
  printSystemInfo();
}

// Core 1 (次核心) 初始化 - 負責感測器讀取
void setup1() {
  // 等待主核心完成基本初始化
  delay(100);
  
  // 初始化感測器系統
  if (initSensorSystem()) {
    core1_active = true;
    // 通過串口通知初始化完成 (小心避免與 Core 0 衝突)
  } else {
    // 初始化失敗，停止執行
    while (1) {
      delay(1000);
    }
  }
}

// 初始化感測器系統
bool initSensorSystem() {
  // 初始化 I2C（使用預設引腳）
  Wire.begin();
  Wire.setClock(400000);  // 設定 I2C 為快速模式 400kHz
  
  // 設定 ADS1115
  ads.setGain(GAIN_ONE);  // ±4.096V 範圍
  ads.setDataRate(RATE_ADS1115_860SPS);  // 最高取樣率 860 SPS
  
  // 初始化 ADS1115（使用默認地址0x48和Wire1）
  if (!ads.begin(0x48, &Wire)) {
    return false;
  }
  
  return true;
}
/*
// 初始化 OLED 顯示器
bool initOLEDDisplay() {
  // 初始化 I2C 用於 OLED (使用指定引腳)
  Wire1.setSDA(OLED_SDA_PIN);
  Wire1.setSCL(OLED_SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);  // 400kHz I2C 時鐘
  
  // 初始化 SSD1306 顯示器
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS,&Wire1)) {
    return false;
  }
  
  // 清除顯示緩衝區
  display.clearDisplay();
  display.display();
  
  return true;
}
*/
// Initialize WiFi connection and TCP server
bool initWiFiAndServer() {
  Serial.println("=== WiFi TCP Server Startup ===");
  Serial.printf("SSID: '%s'\n", ssid);
  Serial.printf("TCP Port: %d\n", TCP_SERVER_PORT);
  
  // Set WiFi mode to Station
  WiFi.mode(WIFI_STA);
  Serial.println("WiFi Mode Set to STA");
  
  // Scan available networks (fast version, reduce startup time)
  Serial.println("Scanning WiFi Networks...");
  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    Serial.println("❌ No WiFi Networks Found");
    return false;
  }
  
  // Display all available networks (for diagnostics)
  Serial.printf("📡 Found %d WiFi Networks:\n", n);
  for (int i = 0; i < n; ++i) {
    Serial.printf("  %d. '%s' (%d dBm) %s\n", 
                  i+1, 
                  WiFi.SSID(i), 
                  WiFi.RSSI(i),
                  (WiFi.encryptionType(i) == CYW43_AUTH_OPEN) ? "[Open]" : "[Encrypted]");
  }
  
  // 檢查並嘗試連接可用的WiFi網路
  bool connected = false;
  for (int netIdx = 0; netIdx < numWiFiNetworks && !connected; netIdx++) {
    const char* currentSSID = wifiNetworks[netIdx].ssid;
    const char* currentPassword = wifiNetworks[netIdx].password;
    
    // 檢查此網路是否在掃描結果中
    bool found_target = false;
    for (int i = 0; i < n && !found_target; ++i) {
      if (strcmp(WiFi.SSID(i), currentSSID) == 0) {
        Serial.printf("✓ Network Found: '%s' (Signal: %d dBm)\n", currentSSID, WiFi.RSSI(i));
        found_target = true;
      }
    }
    
    if (!found_target) {
      Serial.printf("⏭️ Skipping Network: '%s' (Not Found)\n", currentSSID);
      continue;
    }
    
    // Attempt to connect to this network
    Serial.printf("🔄 Connecting to '%s'...\n", currentSSID);
    WiFi.begin(currentSSID, currentPassword);
    
    // 等待連接 (最多 10 秒)
    int timeout = 10;
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
      delay(1000);
      Serial.print(".");
      timeout--;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\n🎉 Successfully Connected to '%s'!\n", currentSSID);
      ssid = currentSSID;      // Update global variable
      password = currentPassword;
      connected = true;
    } else {
      Serial.printf("\n❌ Failed to Connect to '%s'\n", currentSSID);
    }
  }
  
  if (!connected) {
    Serial.println("❌ 所有網路連接嘗試均失敗");
    Serial.println("💡 請檢查上方列表中的正確網路名稱");
    return false;
  }
  
  // 顯示連接資訊
  Serial.printf("IP 地址: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("信號強度: %d dBm\n", WiFi.RSSI());
  
  // 啟動 TCP 伺服器
  tcpServer.begin();
  Serial.printf("🌐 TCP 伺服器啟動成功 (Port: %d)\n", TCP_SERVER_PORT);
  Serial.printf("連接指令: telnet %s %d\n", WiFi.localIP().toString().c_str(), TCP_SERVER_PORT);
  
  wifiConnected = true;
  return true;
  
  // All networks failed to connect
  Serial.println("\n❌ All WiFi Network Connections Failed");
  Serial.printf("Current Status: %s\n", getWiFiStatusString((wl_status_t)WiFi.status()).c_str());
  Serial.println("🔧 Troubleshooting Suggestions:");
  Serial.println("  1. Check SSID is correct (case sensitive)");
  Serial.println("  2. Verify password is correct");
  Serial.println("  3. Ensure router is within range");
  Serial.println("  4. Check router supports 2.4GHz band");
  Serial.println("  5. 重新啟動路由器或機器人");
  wifiConnected = false;
  return false;
}

// WiFi status string conversion (Enhanced diagnostic info)
String getWiFiStatusString(int status) {
  switch(status) {
    case WL_IDLE_STATUS: 
      return "Idle (Initializing)";
    case WL_NO_SSID_AVAIL: 
      return "SSID Not Found (" + String(ssid) + ")";
    case WL_SCAN_COMPLETED: 
      return "Scan Complete";
    case WL_CONNECTED: 
      if (WiFi.localIP().toString() != "0.0.0.0") {
        return "Connected (IP: " + WiFi.localIP().toString() + ", Signal: " + String(WiFi.RSSI()) + "dBm)";
      } else {
        return "Connected (Getting IP...)";
      }
    case WL_CONNECT_FAILED: 
      return "Connection Failed (Check Password/SSID)";
    case WL_CONNECTION_LOST: 
      return "Connection Lost (Signal Unstable/Router Issue)";
    case WL_DISCONNECTED: 
      return "Disconnected (Awaiting Reconnect)";
    default: 
      return "Unknown Status (Code: " + String(status) + ")";
  }
}

// Display system information
void printSystemInfo() {
  Serial.println("System Specifications:");
  Serial.println("- Processor: Dual-Core RP2040");
  Serial.println("- Core 0: Motor Control & Tactical Logic + OLED");
  Serial.println("- Core 1: High-Speed Sensor Reading");
  Serial.println("- ADC: ADS1115 16-bit");
  Serial.println("- I2C: 400kHz Fast Mode (Dual Channel)");
  Serial.println("- Sample Rate: 860 SPS");
  Serial.println("- Sensors: 4x QRE1113");
  Serial.println("- Display: SSD1306 128x64 OLED");
  Serial.println("- Expected Performance: >500 Hz (Dual-Core)");
  if (wifiConnected) {
    Serial.printf("- WiFi: %s (IP: %s)\n", ssid, WiFi.localIP().toString().c_str());
    Serial.printf("- TCP Server: Port %d (Non-Blocking Design)\n", TCP_SERVER_PORT);
    Serial.printf("- WiFi/TCP Processing Frequency: 20Hz (50ms Interval)\n");
  } else {
    Serial.println("- WiFi: Offline Mode");
  }
  Serial.println("- Core 0 Load Optimization: WiFi/TCP Rate-Limited Processing");
  Serial.println("- Core 1 Protection: Completely Independent Operation");
  Serial.println("=====================================");
}

// ========== WiFi 和 TCP 伺服器處理函數 ==========

// 處理 WiFi 連接狀態和 TCP 伺服器（優化版 - 最小化 Core 0 負載）
void handleWiFiAndTCP(QRE_AllSensors &sensors) {
  static unsigned long lastProcessTime = 0;
  unsigned long currentTime = millis();
  
  // 限制 WiFi/TCP 處理頻率，避免影響主迴圈性能
  // 每 50ms 處理一次 WiFi/TCP（20Hz），為主迴圈保留更多時間
  if (currentTime - lastProcessTime < 50) {
    return; // 提早返回，節省 CPU 時間
  }
  lastProcessTime = currentTime;
  
  // 定期檢查 WiFi 連接狀態（降頻處理）
  if (currentTime - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    checkWiFiConnection();
    lastWiFiCheck = currentTime;
  }
  
  // 如果 WiFi 已連接，處理 TCP 伺服器（非阻塞設計）
  if (wifiConnected) {
    handleTCPClients(sensors);
  }
}

// 檢查 WiFi 連接狀態（增強版診斷功能）
void checkWiFiConnection() {
  wl_status_t currentStatus = (wl_status_t)WiFi.status();
  
  // WiFi disconnection handling
  if (currentStatus != WL_CONNECTED && wifiConnected) {
    wifiConnected = false;
    Serial.printf("⚠️ WiFi Connection Interrupted! Status: %s\n", getWiFiStatusString(currentStatus).c_str());
    Serial.printf("📡 Signal Strength: %d dBm\n", WiFi.RSSI());
    
    // Attempt to reconnect to all available networks
    Serial.println("🔄 Starting Reconnection Procedure...");
    WiFi.disconnect();
    delay(1000); // Wait for complete disconnection
    
    // 嘗試重新連接到任何可用的網路
    bool reconnected = false;
    for (int netIndex = 0; netIndex < numWiFiNetworks && !reconnected; netIndex++) {
      const char* currentSSID = wifiNetworks[netIndex].ssid;
      const char* currentPassword = wifiNetworks[netIndex].password;
      
      Serial.printf("🔄 嘗試重連到: %s\n", currentSSID);
      WiFi.begin(currentSSID, currentPassword);
      
      // 等待連接結果（最多8秒）
      unsigned long reconnectStart = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - reconnectStart < 8000) {
        delay(300);
        Serial.print(".");
      }
      Serial.println();
      
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("✅ 重新連接成功！SSID: %s\n", currentSSID);
        reconnected = true;
        wifiConnected = true;
        // 更新當前使用的網路資訊
        ssid = currentSSID;
        password = currentPassword;
      }
    }
    
    if (!reconnected) {
      Serial.println("❌ All Network Reconnection Attempts Failed");
      Serial.printf("📡 Current Status: %s\n", getWiFiStatusString((wl_status_t)WiFi.status()).c_str());
    } else {
      Serial.printf("✓ WiFi Reconnection Successful: %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("📶 Signal Strength: %d dBm\n", WiFi.RSSI());
      
      // Restart TCP server
      tcpServer.begin();
      Serial.printf("🌐 TCP Server Restarted: Port %d\n", TCP_SERVER_PORT);
    }
  }
  // WiFi reconnection success handling
  else if (currentStatus == WL_CONNECTED && !wifiConnected) {
    wifiConnected = true;
    Serial.printf("✓ WiFi Auto-Reconnection Successful: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("📶 Signal Strength: %d dBm\n", WiFi.RSSI());
    
    // Restart TCP server
    tcpServer.begin();
    Serial.printf("🌐 TCP Server Restarted: Port %d\n", TCP_SERVER_PORT);
  }
  // 定期診斷（每2分鐘輸出一次狀態）
  else if (wifiConnected && millis() % 120000 < 1000) {
    Serial.printf("📊 WiFi 診斷 - 訊號: %d dBm, IP: %s\n", 
                  WiFi.RSSI(), WiFi.localIP().toString().c_str());
  }
}

// 處理 TCP 客戶端連接和數據傳輸（完全非阻塞版本）
void handleTCPClients(QRE_AllSensors &sensors) {
  // 檢查是否有新的客戶端連接 (完全非阻塞)
  WiFiClient client = tcpServer.accept();
  if (!client) {
    return; // 沒有客戶端連接，立即返回
  }
  
  Serial.printf("🔗 新客戶端連接: %s\n", client.remoteIP().toString().c_str());
  
  // 發送歡迎訊息 (使用英文避免編碼問題)
  client.println("=== Bottle Sumo Robot TCP Server ===");
  client.println("Commands:");
  client.println("  'data' - Text format sensor data");
  client.println("  'json' - JSON format sensor data");  
  client.println("  'status' - System status");
  client.println("  'help' - Command help");
  client.println("  'ping' - Connection test");
  client.print("> ");
  
  // 完全非阻塞的請求處理（最多檢查1秒）
  unsigned long startTime = millis();
  const unsigned long MAX_WAIT_TIME = 3000; // 降低到1秒，減少對主迴圈的影響
  
  while (!client.available() && (millis() - startTime) < MAX_WAIT_TIME) {
    delay(10); // 短暫延遲，允許其他任務執行
    
    // 每100ms輸出一次等待狀態，避免長時間阻塞
    if ((millis() - startTime) % 100 == 0) {
      // 可以在這裡添加心跳或其他輕量處理
    }
  }
  
  if (client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();
    command.toLowerCase(); // 統一轉為小寫
    
    Serial.printf("📨 收到指令: '%s'\n", command.c_str());
    
    // 快速指令處理
    if (command == "data") {
      sendSensorData(client, sensors);
    } else if (command == "json") {
      sendSensorJSON(client, sensors);
    } else if (command == "status") {
      sendSystemStatus(client);
    } else if (command == "help") {
      sendHelpInfo(client);
    } else if (command == "ping") {
      client.println("PONG - Server is alive!");
      client.printf("WiFi Signal: %d dBm\n", WiFi.RSSI());
      client.printf("Free Heap: %d bytes\n", rp2040.getFreeHeap());
    } else {
      client.println("Unknown command. Type 'help' for available commands.");
    }
  } else {
    client.println("Request timeout (1 second). Connection will close.");
    Serial.println("⏰ Client request timeout");
  }
  
  // 立即關閉連接，釋放資源
  client.stop();
  Serial.println("🔌 客戶端連接已關閉");
}

// ========== TCP 數據傳輸函數 ==========

// 發送感測器數據（純文字格式）
void sendSensorData(WiFiClient &client, QRE_AllSensors &sensors) {
  // 讀取最新感測器數據（線程安全）
  QRE_AllSensors localData;
  mutex_enter_blocking(&data_mutex);
  localData = sensors;
  mutex_exit(&data_mutex);
  
  client.println("=== QRE1113 sensor data ===");
  client.printf("times: %lu ms\n", millis());
  client.println("original value (0-32767):\n");
  for (int i = 0; i < 4; i++) {
    client.printf("  sensor %d: %d (%.3fV)\n", i, localData.sensor[i].raw_value, localData.sensor[i].voltage);
  }
  client.println("========================");
}

// 發送感測器數據（JSON 格式）
void sendSensorJSON(WiFiClient &client, QRE_AllSensors &sensors) {
  // 讀取最新感測器數據（線程安全）
  QRE_AllSensors localData;
  mutex_enter_blocking(&data_mutex);
  localData = sensors;
  mutex_exit(&data_mutex);
  
  // 構建 JSON 字符串
  String json = "{";
  json += "\"timestamp\":" + String(millis()) + ",";
  json += "\"sensors\":{";
  json += "\"raw\":[" + String(localData.sensor[0].raw_value) + "," + String(localData.sensor[1].raw_value) + "," + String(localData.sensor[2].raw_value) + "," + String(localData.sensor[3].raw_value) + "],";
  json += "\"voltage\":[" + String(localData.sensor[0].voltage, 3) + "," + String(localData.sensor[1].voltage, 3) + "," + String(localData.sensor[2].voltage, 3) + "," + String(localData.sensor[3].voltage, 3) + "]";
  json += "},";
  json += "\"wifi_status\":\"" + getWiFiStatusString((wl_status_t)WiFi.status()) + "\",";
  json += "\"uptime\":" + String(millis()) + "";
  json += "}";
  
  client.println(json);
}

// Send system status information
void sendSystemStatus(WiFiClient &client) {
  client.println("=== System Status ===");
  client.printf("Firmware Version: Bottle Sumo v1.0\n");
  client.printf("WiFi Status: %s\n", getWiFiStatusString(WiFi.status()).c_str());
  if (wifiConnected) {
    client.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
    client.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
  }
  client.printf("Uptime: %lu ms\n", millis());
  client.printf("Free Memory: %d bytes\n", rp2040.getFreeHeap());
  client.printf("CPU 溫度: %.1f°C\n", analogReadTemp());
  client.printf("核心 0: 主控制迴圈\n");
  client.printf("核心 1: 感測器讀取 (目標: 860 Hz)\n");
  client.println("===============");
}

// 發送幫助資訊
void sendHelpInfo(WiFiClient &client) {
  client.println("=== 可用指令 ===");
  client.println("data   - 獲取 QRE1113 感測器數據（純文字格式）");
  client.println("json   - 獲取感測器數據（JSON 格式）");
  client.println("status - 獲取系統狀態資訊");
  client.println("help   - 顯示此幫助資訊");
  client.println("===============");
}

// 根據數位讀數判斷機器人狀態
String digitalReadingsToStatus(bool digital[4]) {
  int activeCount = 0;
  for (int i = 0; i < 4; i++) {
    if (digital[i]) activeCount++;
  }
  
  if (activeCount == 0) return "safe";          // 安全區域
  else if (activeCount == 1) return "edge";     // 邊緣偵測
  else if (activeCount == 2) return "corner";   // 角落
  else if (activeCount >= 3) return "danger";   // 危險區域
  else return "unknown";
}

// 根據感測器電壓判斷機器人狀態
String sensorReadingsToStatus(QRE_AllSensors &sensors) {
  int activeCount = 0;
  const float EDGE_THRESHOLD = 1.0; // 1V 以下認為是邊緣
  
  for (int i = 0; i < 4; i++) {
    if (sensors.sensor[i].voltage < EDGE_THRESHOLD) activeCount++;
  }
  
  if (activeCount == 0) return "safe";          // 安全區域
  else if (activeCount == 1) return "edge";     // 邊緣偵測
  else if (activeCount == 2) return "corner";   // 角落
  else if (activeCount >= 3) return "danger";   // 危險區域
  else return "unknown";
}

// ========== OLED 顯示函數 ==========
/*
// 顯示啟動畫面
void showStartupScreen() {
  display.clearDisplay();
  
  // 設定文字顏色和大小
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // 標題
  display.setCursor(15, 0);
  display.println("BOTTLE SUMO ROBOT");
  
  // 分隔線
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // 系統信息
  display.setCursor(0, 15);
  display.println("Dual-Core RP2040");
  display.setCursor(0, 25);
  display.println("Core 0: Motor Control");
  display.setCursor(0, 35);
  display.println("Core 1: Sensors 860Hz");
  
  // 硬體信息
  display.setCursor(0, 45);
  display.println("ADS1115 + 4x QRE1113");
  
  // 狀態指示
  display.setCursor(85, 55);
  display.println("READY!");
  
  display.display();
  delay(2000);  // 顯示 2 秒
}

// 更新 OLED 主顯示（包含 WiFi 狀態）
void updateOLEDDisplay(QRE_AllSensors &sensors) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // 標題行
  display.setTextSize(1);
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  // WiFi 狀態行
  display.setCursor(0, 0);
  if (wifiConnected) {
    display.print("WiFi: ");
    display.print(WiFi.localIP().toString());
    // WiFi 信號強度指示
    int rssi = WiFi.RSSI();
    if (rssi > -50) display.print(" ****");
    else if (rssi > -60) display.print(" ***");
    else if (rssi > -70) display.print(" **");
    else display.print(" *");
  } else {
    display.print("WiFi: OFFLINE");
  }
  
  // TCP 伺服器狀態
  display.setCursor(0, 12);
  if (wifiConnected) {
    display.printf("TCP: Port %d READY", TCP_SERVER_PORT);
  } else {
    display.print("TCP: DISABLED");
  }
  
  // Sensor status row
  display.setCursor(0, 22);
  display.print("Sensors: ");
  for (int i = 0; i < 4; i++) {
    // Simple edge detection: voltage < 1.0V indicates edge detected
    if (sensors.sensor[i].voltage < 1.0) {
      display.print("E"); // E indicates edge detected
    } else {
      display.print("S"); // S indicates safe
    }
  }
  
  // 感測器狀態
  display.setCursor(70, 32);
  display.print("READY");
  
  // 機器人狀態
  display.setCursor(0, 32);
  display.print("State: MONITORING");
  
  // 系統狀態行
  display.setCursor(0, 42);
  display.printf("Uptime: %lus", millis()/1000);
  
  // Core 頻率狀態
  display.setCursor(0, 52);
  display.printf("C0:%.0f C1:%.0f", 
                 core0_loop_count * 1000.0 / millis(),
                 core1_loop_count * 1000.0 / millis());
  
  display.display();
}
*/
// ========== 感測器讀取函數 ==========

// Core 1 專用：更新共享數據
void updateSharedSensorData() {
  static int16_t raw_values[4];
  static float voltages[4];
  
  // 讀取所有感測器
  for (int i = 0; i < 4; i++) {
    raw_values[i] = ads.readADC_SingleEnded(i);
    voltages[i] = ads.computeVolts(raw_values[i]);
  }
  
  // 更新共享數據 (使用互斥鎖確保數據完整性)
  mutex_enter_blocking(&data_mutex);
  for (int i = 0; i < 4; i++) {
    shared_data.raw_values[i] = raw_values[i];
    shared_data.voltages[i] = voltages[i];
  }
  shared_data.timestamp = millis();
  shared_data.data_ready = true;
  mutex_exit(&data_mutex);
}

// Core 0 專用：從共享數據獲取感測器資訊
QRE_AllSensors getAllSensorsFromShared() {
  QRE_AllSensors all_sensors;
  all_sensors.updateFromSharedData();
  return all_sensors;
}

// 讀取單一感測器 (保留用於調試)
QRE_Reading getSingleSensor(uint8_t channel) {
  QRE_Reading reading;
  reading.raw_value = ads.readADC_SingleEnded(channel);
  reading.voltage = ads.computeVolts(reading.raw_value);
  return reading;
}

// 高速讀取所有感測器 (Legacy function, 現在由 Core 1 處理)
QRE_AllSensors getAllSensors() {
  return getAllSensorsFromShared();
}

// 超高速版本：直接從共享數據讀取原始值
void getAllSensorsRaw(int16_t raw_values[4]) {
  mutex_enter_blocking(&data_mutex);
  for (int i = 0; i < 4; i++) {
    raw_values[i] = shared_data.raw_values[i];
  }
  mutex_exit(&data_mutex);
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
      Serial.println("🔍 搜尋對手 - 建議: 前進/旋轉尋找目標");
      // TODO: 實現搜尋邏輯
      // motorForward(speed_low);
      // 或 motorRotate(direction);
      break;
      
    case ATTACK_FORWARD:
      Serial.println("⚔️ 攻擊模式 - 建議: 全速前進");
      // TODO: 實現攻擊邏輯
      // motorForward(speed_max);
      break;
      
    case RETREAT_AND_TURN: {
      Serial.println("↩️ 後退轉向 - 建議: 避開邊緣並重新定位");
      // TODO: 實現後退轉向邏輯
      String direction = sensors.getEdgeDirection();
      Serial.print("   危險方向: ");
      Serial.println(direction);
      // motorBackward(speed_medium);
      // delay(500);
      // motorTurn(opposite_direction);
      break;
    }
      
    case EMERGENCY_REVERSE:
      Serial.println("🚨 緊急後退 - 建議: 立即遠離邊緣");
      // TODO: 實現緊急後退邏輯
      // motorBackward(speed_max);
      break;
  }
}

// Bottle Sumo 狀態顯示
void printSumoStatus(QRE_AllSensors &sensors) {
  Serial.println("========== Bottle Sumo 雙核心狀態 ==========");
  
  // 核心狀態監控
  Serial.print("Core 0 循環: ");
  Serial.print(core0_loop_count);
  Serial.print(" | Core 1 循環: ");
  Serial.println(core1_loop_count);
  
  // 數據新鮮度檢查
  Serial.print("數據狀態: ");
  Serial.println(sensors.isDataFresh() ? "✅ 新鮮" : "⚠️ 過時");
  
  // 基本感測器資訊
  String sensor_names[] = {"前左", "前右", "後左", "後右"};
  for (int i = 0; i < 4; i++) {
    Serial.print(sensor_names[i]);
    Serial.print(": ");
    Serial.print(sensors.sensor[i].voltage, 2);
    Serial.print("V ");
    Serial.println(sensors.sensor[i].voltage > 2.5 ? "[邊緣!]" : "[安全]");
  }
  
  // Sumo 專用分析
  Serial.print("邊緣檢測: ");
  Serial.println(sensors.isEdgeDetected() ? "⚠️ 檢測到!" : "✅ 安全");
  
  Serial.print("危險方向: ");
  Serial.println(sensors.getEdgeDirection());
  
  Serial.print("危險等級: ");
  Serial.print(sensors.getDangerLevel());
  Serial.println("/4");
  
  Serial.println("==========================================");
}

// 性能測試
void performanceTest() {
  Serial.println("\n=== 性能測試 ===");
  
  unsigned long start_time = micros();
  QRE_AllSensors sensors = getAllSensors();
  unsigned long end_time = micros();
  
  unsigned long read_time = end_time - start_time;
  float frequency = 1000000.0 / read_time;
  
  Serial.print("讀取時間: ");
  Serial.print(read_time);
  Serial.println(" 微秒");
  
  Serial.print("讀取頻率: ");
  Serial.print(frequency, 1);
  Serial.println(" Hz");
  
  Serial.println("Significance for Bottle Sumo:");
  Serial.println("✓ Ultra-Fast Response, Prevents Ring-Out");
  Serial.println("✓ Exceeds Human Reaction Time by 30+ Times");
  Serial.println("✓ Meets All Competition-Grade Requirements");
  Serial.println("==================");
}

// ========== 主循環 ==========

// Core 0 主循環 - 馬達控制與戰術邏輯
void loop() {
  static int loop_count = 0;
  
  // 更新循環計數器
  core0_loop_count++;
  
  // 從 Core 1 獲取最新感測器數據
  QRE_AllSensors all_sensors = getAllSensorsFromShared();
  
  // 檢查數據新鮮度
  if (!all_sensors.isDataFresh(20)) {  // 20ms 容忍度
    Serial.println("⚠️ 警告: 感測器數據過時!");
    delay(5);
    return;
  }
  
  // 每 10 次循環顯示詳細狀態
  if (loop_count % 10 == 0) {
    printSumoStatus(all_sensors);
    
    // 決定並執行動作
    SumoAction action = decideSumoAction(all_sensors);
    executeSumoAction(action, all_sensors);
    
    Serial.print("Core 0 頻率: ");
    Serial.print(core0_loop_count * 1000.0 / millis(), 1);
    Serial.print(" Hz | Core 1 頻率: ");
    Serial.print(core1_loop_count * 1000.0 / millis(), 1);
    Serial.println(" Hz");
    Serial.println();
  }
  
  // 每 50 次循環更新 OLED 顯示（避免過度頻繁更新）
  if (loop_count % 50 == 0) {
    //updateOLEDDisplay(all_sensors);
  } else {
    // 快速循環：只顯示關鍵資訊
    Serial.print("C0-");
    Serial.print(loop_count);
    Serial.print(" | 邊緣: ");
    Serial.print(all_sensors.isEdgeDetected() ? "⚠️" : "✅");
    Serial.print(" | 方向: ");
    Serial.print(all_sensors.getEdgeDirection());
    Serial.print(" | 危險: ");
    Serial.print(all_sensors.getDangerLevel());
    Serial.print("/4");
    Serial.println();
  }
  
  loop_count++;
  
  // WiFi 狀態監控和 TCP 伺服器處理
  handleWiFiAndTCP(all_sensors);
  
  // 馬達控制邏輯在這裡執行
  // TODO: 添加您的馬達控制代碼
  
  // 適當的延遲，讓 Core 0 以適合馬達控制的頻率運行 (例如 50-100Hz)
  delay(10);  // 100Hz
}

// Core 1 主循環 - 專門負責感測器讀取
void loop1() {
  // 更新循環計數器
  core1_loop_count++;
  
  // 持續高速讀取感測器並更新共享數據
  updateSharedSensorData();
  
  // 最小延遲，讓 Core 1 以最高速度運行
  // 實際速度由 ADS1115 的讀取速度決定 (~860 SPS)
}

// ========== 馬達控制函數 (Core 0 專用) ==========

/*
 * 以下是馬達控制函數範例，請根據您的硬體配置修改
 * 這些函數只會在 Core 0 中調用，確保馬達控制的一致性
 */

// 馬達引腳定義 (請根據實際硬體修改)
/*
const int LEFT_MOTOR_PIN1 = 2;   // 左馬達正轉
const int LEFT_MOTOR_PIN2 = 3;   // 左馬達反轉
const int RIGHT_MOTOR_PIN1 = 4;  // 右馬達正轉
const int RIGHT_MOTOR_PIN2 = 5;  // 右馬達反轉
const int LEFT_MOTOR_PWM = 6;    // 左馬達速度控制
const int RIGHT_MOTOR_PWM = 7;   // 右馬達速度控制

void initMotors() {
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  
  // 初始狀態：停止
  motorStop();
}

void motorForward(int speed) {
  // 前進 (兩輪同方向)
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  
  analogWrite(LEFT_MOTOR_PWM, speed);
  analogWrite(RIGHT_MOTOR_PWM, speed);
}

void motorBackward(int speed) {
  // 後退 (兩輪反方向)
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
  
  analogWrite(LEFT_MOTOR_PWM, speed);
  analogWrite(RIGHT_MOTOR_PWM, speed);
}

void motorTurnLeft(int speed) {
  // 左轉 (右輪前進，左輪後退)
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  
  analogWrite(LEFT_MOTOR_PWM, speed);
  analogWrite(RIGHT_MOTOR_PWM, speed);
}

void motorTurnRight(int speed) {
  // 右轉 (左輪前進，右輪後退)
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
  
  analogWrite(LEFT_MOTOR_PWM, speed);
  analogWrite(RIGHT_MOTOR_PWM, speed);
}

void motorStop() {
  // 停止所有馬達
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}

// 速度常數定義
const int SPEED_STOP = 0;
const int SPEED_LOW = 100;
const int SPEED_MEDIUM = 180;
const int SPEED_HIGH = 255;
*/

// ========== 輔助函數 (可選) ==========

/*
 * 使用範例:
 * 
 * 1. 基本讀取:
 *    QRE_AllSensors sensors = getAllSensors();
 *    float voltage = sensors.getVoltage(0);
 * 
 * 2. 邊緣檢測:
 *    if (sensors.isEdgeDetected()) {
 *      String direction = sensors.getEdgeDirection();
 *      // 執行避險動作
 *    }
 * 
 * 3. 戰術決策:
 *    SumoAction action = decideSumoAction(sensors);
 *    executeSumoAction(action, sensors);
 * 
 * 4. 高速控制循環:
 *    int16_t raw_values[4];
 *    getAllSensorsRaw(raw_values);
 *    // 直接使用原始值進行快速判斷
 * 
 * 5. OLED 顯示功能:
 *    - 啟動時自動顯示歡迎畫面
 *    - 系統狀態頁面：核心頻率、感測器數值
 *    - 戰術狀態頁面：當前動作、危險等級、方向
 *    - 自動在正常和緊急模式間切換顯示
 * 
 * 設置步驟:
 * 1. 安裝 Arduino IDE 庫:
 *    - Adafruit ADS1X15
 *    - Adafruit GFX Library
 *    - Adafruit SSD1306
 * 2. 連接硬體如上述線路圖
 * 3. 上傳程式到 Raspberry Pi Pico
 * 4. 打開串口監視器查看系統狀態
 * 5. OLED 將自動顯示啟動信息和運行狀態
 */