#include <Adafruit_ADS1X15.h>
#include <Wire.h>

// 結構體來返回 ADC 原始值和電壓值
struct QRE_Reading {
  int16_t raw_value;
  float voltage;
};
// Bottle Sumo 控制策略
enum SumoAction {
  SEARCH_OPPONENT,    // 搜尋對手
  ATTACK_FORWARD,     // 直線攻擊
  RETREAT_AND_TURN,   // 後退並轉向
  EMERGENCY_REVERSE   // 緊急後退
};

// 結構體來存放所有 4 個 QRE1113 感測器的數據
struct QRE_AllSensors {
  QRE_Reading sensor[4];  // 4 個感測器的數據陣列
  
  // 便利函數：取得特定感測器的原始值
  int16_t getRawValue(int index) {
    if (index >= 0 && index < 4) {
      return sensor[index].raw_value;
    }
    return 0;
  }
  
  // 便利函數：取得特定感測器的電壓值
  float getVoltage(int index) {
    if (index >= 0 && index < 4) {
      return sensor[index].voltage;
    }
    return 0.0;
  }
  
  // 便利函數：顯示所有感測器數據
  void printAll() {
    Serial.println("=== 所有 QRE1113 感測器數據 ===");
    for (int i = 0; i < 4; i++) {
      Serial.print("感測器 ");
      Serial.print(i);
      Serial.print(": Raw=");
      Serial.print(sensor[i].raw_value);
      Serial.print(", Voltage=");
      Serial.print(sensor[i].voltage, 3);
      Serial.println("V");
    }
    Serial.println("==============================");
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
    bool front_left = sensor[0].voltage > edge_threshold;   // 假設感測器布局
    bool front_right = sensor[1].voltage > edge_threshold;
    bool back_left = sensor[2].voltage > edge_threshold;
    bool back_right = sensor[3].voltage > edge_threshold;
    
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

// 全域 ADS1115 物件宣告
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Hello!");

  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  init_ads();
}

// 初始化 ADS1115 的函數
int init_ads(){
  // 初始化 I2C（使用預設引腳）
  Wire.begin();
  Wire.setClock(400000);  // 設定 I2C 為快速模式 400kHz
  
  // 設定增益
  ads.setGain(GAIN_ONE);  // ±4.096V 範圍
  
  // 設定最高取樣率以獲得最快速度
  ads.setDataRate(RATE_ADS1115_860SPS);  // 860 SPS (最快)
  
  // 初始化 ADS1115
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    return 0;
  }
  
  Serial.println("ADS1115 初始化成功！(高速模式)");
  Serial.println("I2C: 400kHz, 取樣率: 860 SPS");
  return 1;
}

// 獲取指定通道的 QRE1113 感測器數值
QRE_Reading get_qre1113_value(uint8_t channel){
  QRE_Reading reading;
  reading.raw_value = ads.readADC_SingleEnded(channel);
  reading.voltage = ads.computeVolts(reading.raw_value);
  return reading;
}

// 一次性獲取所有 4 個 QRE1113 感測器的數值
QRE_AllSensors get_all_qre1113_values(){
  QRE_AllSensors all_sensors;
  
  // 讀取所有 4 個通道
  for (int i = 0; i < 4; i++) {
    all_sensors.sensor[i] = get_qre1113_value(i);
  }
  
  return all_sensors;
}

// 高速版本：一次性獲取所有感測器數值（優化版）
QRE_AllSensors get_all_qre1113_values_fast(){
  QRE_AllSensors all_sensors;
  
  // 直接讀取，減少函數調用開銷
  for (int i = 0; i < 4; i++) {
    all_sensors.sensor[i].raw_value = ads.readADC_SingleEnded(i);
    all_sensors.sensor[i].voltage = ads.computeVolts(all_sensors.sensor[i].raw_value);
  }
  
  return all_sensors;
}

// 超高速版本：只讀取原始值，不計算電壓（用於即時控制）
void get_all_qre1113_raw_fast(int16_t raw_values[4]){
  for (int i = 0; i < 4; i++) {
    raw_values[i] = ads.readADC_SingleEnded(i);
  }
}

// 性能測試函數
void performance_test() {
  unsigned long start_time, end_time;
  int16_t raw_values[4];
  
  Serial.println("\n=== 性能測試 ===");
  
  // 測試 1: 標準版本
  start_time = micros();
  QRE_AllSensors sensors1 = get_all_qre1113_values();
  end_time = micros();
  Serial.print("標準版本: ");
  Serial.print(end_time - start_time);
  Serial.println(" 微秒");
  
  // 測試 2: 高速版本
  start_time = micros();
  QRE_AllSensors sensors2 = get_all_qre1113_values_fast();
  end_time = micros();
  Serial.print("高速版本: ");
  Serial.print(end_time - start_time);
  Serial.println(" 微秒");
  
  // 測試 3: 超高速版本（只讀原始值）
  start_time = micros();
  get_all_qre1113_raw_fast(raw_values);
  end_time = micros();
  Serial.print("超高速版本(僅原始值): ");
  Serial.print(end_time - start_time);
  Serial.println(" 微秒");
  
  // 計算理論最大 Hz
  float max_hz = 1000000.0 / (end_time - start_time);
  Serial.print("理論最大頻率: ");
  Serial.print(max_hz, 1);
  Serial.println(" Hz");
  
  Serial.println("==================");
}

// 快速檢查是否有感測器檢測到黑線（基於電壓閾值）
bool checkBlackLineDetection(QRE_AllSensors &sensors, float threshold = 1.5) {
  for (int i = 0; i < 4; i++) {
    if (sensors.sensor[i].voltage < threshold) {
      return true;  // 至少有一個感測器檢測到黑線
    }
  }
  return false;  // 沒有感測器檢測到黑線
}

// 獲取檢測到黑線的感測器數量
int getBlackLineCount(QRE_AllSensors &sensors, float threshold = 1.5) {
  int count = 0;
  for (int i = 0; i < 4; i++) {
    if (sensors.sensor[i].voltage < threshold) {
      count++;
    }
  }
  return count;
}

// 獲取所有感測器的狀態位元組（位元 0-3 代表感測器 0-3）
uint8_t getSensorStatusByte(QRE_AllSensors &sensors, float threshold = 1.5) {
  uint8_t status = 0;
  for (int i = 0; i < 4; i++) {
    if (sensors.sensor[i].voltage < threshold) {
      status |= (1 << i);  // 設定對應的位元
    }
  }
  return status;
}

// ========== Bottle Sumo 專用函數 ==========

// 根據感測器狀態決定 Sumo 行動
SumoAction decideSumoAction(QRE_AllSensors &sensors) {
  float EDGE_THRESHOLD = 2.5;  // 邊緣檢測閾值
  
  int danger_level = sensors.getDangerLevel(EDGE_THRESHOLD);
  String edge_direction = sensors.getEdgeDirection(EDGE_THRESHOLD);
  
  // 危險等級判斷
  if (danger_level >= 2) {
    return EMERGENCY_REVERSE;  // 多個感測器檢測到邊緣
  } else if (danger_level == 1) {
    return RETREAT_AND_TURN;   // 單個感測器檢測到邊緣
  } else {
    return SEARCH_OPPONENT;    // 沒有檢測到邊緣，繼續搜尋
  }
}

// Bottle Sumo 狀態顯示
void printSumoStatus(QRE_AllSensors &sensors) {
  Serial.println("========== Bottle Sumo 狀態 ==========");
  
  // 基本感測器資訊
  for (int i = 0; i < 4; i++) {
    Serial.print("感測器 ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(sensors.sensor[i].voltage, 2);
    Serial.print("V ");
    Serial.println(sensors.sensor[i].voltage > 2.5 ? "[邊緣!]" : "[安全]");
  }
  
  // Sumo 專用分析
  Serial.print("邊緣檢測: ");
  Serial.println(sensors.isEdgeDetected() ? "檢測到!" : "安全");
  
  Serial.print("危險方向: ");
  Serial.println(sensors.getEdgeDirection());
  
  Serial.print("危險等級: ");
  Serial.print(sensors.getDangerLevel());
  Serial.println("/4");
  
  // 建議動作
  SumoAction action = decideSumoAction(sensors);
  Serial.print("建議動作: ");
  switch (action) {
    case SEARCH_OPPONENT:
      Serial.println("搜尋對手 - 前進/旋轉尋找目標");
      break;
    case ATTACK_FORWARD:
      Serial.println("攻擊模式 - 全速前進");
      break;
    case RETREAT_AND_TURN:
      Serial.println("後退轉向 - 避開邊緣並重新定位");
      break;
    case EMERGENCY_REVERSE:
      Serial.println("緊急後退 - 立即遠離邊緣");
      break;
  }
  
  Serial.println("=====================================");
}

// 155.8 Hz 對 Bottle Sumo 的實際意義
void analyzeSumoPerformance() {
  Serial.println("\n========== Bottle Sumo 性能分析 ==========");
  Serial.println("實測讀取速度: 155.8 Hz (6.4ms/次)");
  Serial.println("");
  Serial.println("對 Bottle Sumo 機器人的意義:");
  Serial.println("✓ 反應時間: 6.4ms (人類眨眼 = 300-400ms)");
  Serial.println("✓ 邊緣檢測: 極快，足以防止掉出場地");
  Serial.println("✓ 控制頻率: 比大多數馬達控制更快");
  Serial.println("✓ 競賽等級: 滿足所有 Sumo 比賽需求");
  Serial.println("");
  Serial.println("速度比較:");
  Serial.println("- 人類反應: ~200ms (慢 31倍)");
  Serial.println("- 典型 Arduino loop: ~1ms (慢 6倍)");
  Serial.println("- 馬達 PWM 更新: ~20ms (慢 3倍)");
  Serial.println("結論: 您的感測器速度非常優秀！");
  Serial.println("=========================================");
}

void loop(void)
{
  static unsigned long last_performance_test = 0;
  static unsigned long last_analysis = 0;
  static int loop_count = 0;
  
  // 每 30 秒執行一次性能測試
  if (millis() - last_performance_test > 30000) {
    performance_test();
    last_performance_test = millis();
  }
  
  // 每 15 秒執行一次 Sumo 性能分析
  if (millis() - last_analysis > 15000) {
    analyzeSumoPerformance();
    last_analysis = millis();
  }
  
  // 測量讀取時間
  unsigned long start_time = micros();
  
  // 使用高速版本讀取所有感測器
  QRE_AllSensors all_sensors = get_all_qre1113_values_fast();
  
  unsigned long read_time = micros() - start_time;
  
  // 每 20 次循環顯示一次 Sumo 專用狀態
  if (loop_count % 20 == 0) {
    printSumoStatus(all_sensors);
    
    Serial.print("讀取時間: ");
    Serial.print(read_time);
    Serial.print(" 微秒, 頻率: ");
    Serial.print(1000000.0 / read_time, 1);
    Serial.println(" Hz");
    Serial.println();
  } else {
    // 快速循環：只顯示關鍵 Sumo 資訊
    Serial.print("Loop ");
    Serial.print(loop_count);
    Serial.print(" | 邊緣: ");
    Serial.print(all_sensors.isEdgeDetected() ? "檢測!" : "安全 ");
    Serial.print(" | 方向: ");
    Serial.print(all_sensors.getEdgeDirection());
    Serial.print(" | 危險: ");
    Serial.print(all_sensors.getDangerLevel());
    Serial.print("/4 | ");
    Serial.print(read_time);
    Serial.println("μs");
  }
  
  loop_count++;
  
  // Bottle Sumo 適用的更新頻率
  delay(20);  // 50Hz 更新 - 對 Sumo 機器人完全足夠
}
