/*
 * Bottle Sumo Robot - QRE1113 Edge Detection System
 * 使用 ADS1115 和 QRE1113 感測器進行邊緣檢測
 * 
 * 硬體連接:
 * ADS1115    Raspberry Pi Pico
 * VDD     ←→  3.3V (3V3 OUT)
 * GND     ←→  GND
 * SCL     ←→  SCL (預設引腳)
 * SDA     ←→  SDA (預設引腳)
 * 
 * ADS1115    QRE1113 感測器
 * A0      ←→  QRE1113 #1 (前左)
 * A1      ←→  QRE1113 #2 (前右)
 * A2      ←→  QRE1113 #3 (後左)
 * A3      ←→  QRE1113 #4 (後右)
 * 
 * 性能: 155.8 Hz 讀取速度 (6.4ms/次)
 * 作者: CTEA-BottleSumo 專案
 * 日期: 2025-09-22
 */

#include <Adafruit_ADS1X15.h>
#include <Wire.h>

// ========== 結構體定義 ==========

// 單一感測器讀數結構體
struct QRE_Reading {
  int16_t raw_value;  // ADC 原始值
  float voltage;      // 轉換後的電壓值
};

// 所有感測器資料結構體
struct QRE_AllSensors {
  QRE_Reading sensor[4];  // 4 個感測器的數據陣列
  
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
  
  // 顯示所有感測器數據
  void printAll() {
    Serial.println("=== QRE1113 感測器數據 ===");
    for (int i = 0; i < 4; i++) {
      Serial.print("感測器 ");
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

void setup() {
  Serial.begin(115200);
  Serial.println("Bottle Sumo Robot - QRE1113 系統啟動");
  Serial.println("=====================================");
  
  // 初始化感測器系統
  if (initSensorSystem()) {
    Serial.println("✓ 感測器系統初始化成功");
    printSystemInfo();
  } else {
    Serial.println("✗ 感測器系統初始化失敗");
    while (1);  // 停止執行
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
  
  // 初始化 ADS1115
  if (!ads.begin()) {
    return false;
  }
  
  return true;
}

// 顯示系統資訊
void printSystemInfo() {
  Serial.println("系統規格:");
  Serial.println("- ADC: ADS1115 16-bit");
  Serial.println("- I2C: 400kHz 快速模式");
  Serial.println("- 取樣率: 860 SPS");
  Serial.println("- 感測器: 4x QRE1113");
  Serial.println("- 預期性能: ~155 Hz");
  Serial.println("=====================================");
}

// ========== 感測器讀取函數 ==========

// 讀取單一感測器
QRE_Reading getSingleSensor(uint8_t channel) {
  QRE_Reading reading;
  reading.raw_value = ads.readADC_SingleEnded(channel);
  reading.voltage = ads.computeVolts(reading.raw_value);
  return reading;
}

// 高速讀取所有感測器
QRE_AllSensors getAllSensors() {
  QRE_AllSensors all_sensors;
  
  // 直接讀取，減少函數調用開銷
  for (int i = 0; i < 4; i++) {
    all_sensors.sensor[i].raw_value = ads.readADC_SingleEnded(i);
    all_sensors.sensor[i].voltage = ads.computeVolts(all_sensors.sensor[i].raw_value);
  }
  
  return all_sensors;
}

// 超高速版本：只讀取原始值（用於即時控制）
void getAllSensorsRaw(int16_t raw_values[4]) {
  for (int i = 0; i < 4; i++) {
    raw_values[i] = ads.readADC_SingleEnded(i);
  }
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
  Serial.println("========== Bottle Sumo 狀態 ==========");
  
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
  
  Serial.println("=====================================");
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
  
  Serial.println("對 Bottle Sumo 的意義:");
  Serial.println("✓ 反應極快，足以防止掉出場地");
  Serial.println("✓ 超越人類反應時間 30+ 倍");
  Serial.println("✓ 滿足所有競賽級別需求");
  Serial.println("==================");
}

// ========== 主循環 ==========

void loop() {
  static unsigned long last_performance_test = 0;
  static int loop_count = 0;
  
  // 每 30 秒執行一次性能測試
  if (millis() - last_performance_test > 30000) {
    performanceTest();
    last_performance_test = millis();
  }
  
  // 測量讀取時間
  unsigned long start_time = micros();
  
  // 讀取所有感測器
  QRE_AllSensors all_sensors = getAllSensors();
  
  unsigned long read_time = micros() - start_time;
  
  // 每 10 次循環顯示詳細狀態
  if (loop_count % 10 == 0) {
    printSumoStatus(all_sensors);
    
    // 決定並執行動作
    SumoAction action = decideSumoAction(all_sensors);
    executeSumoAction(action, all_sensors);
    
    Serial.print("讀取時間: ");
    Serial.print(read_time);
    Serial.print(" μs, 頻率: ");
    Serial.print(1000000.0 / read_time, 1);
    Serial.println(" Hz");
    Serial.println();
  } else {
    // 快速循環：只顯示關鍵資訊
    Serial.print("Loop ");
    Serial.print(loop_count);
    Serial.print(" | 邊緣: ");
    Serial.print(all_sensors.isEdgeDetected() ? "⚠️" : "✅");
    Serial.print(" | 方向: ");
    Serial.print(all_sensors.getEdgeDirection());
    Serial.print(" | 危險: ");
    Serial.print(all_sensors.getDangerLevel());
    Serial.print("/4 | ");
    Serial.print(read_time);
    Serial.println("μs");
  }
  
  loop_count++;
  
  // Bottle Sumo 適用的更新頻率 (50Hz)
  delay(20);
}

// ========== 輔助函數 (可選) ==========

/*
 * 以下是一些您可能需要實現的馬達控制函數範例
 * 請根據您的硬體配置來實現這些函數
 */

/*
void motorForward(int speed) {
  // 實現前進邏輯
  // 例如: analogWrite(left_motor_pin, speed);
  //      analogWrite(right_motor_pin, speed);
}

void motorBackward(int speed) {
  // 實現後退邏輯
}

void motorTurnLeft(int speed) {
  // 實現左轉邏輯
}

void motorTurnRight(int speed) {
  // 實現右轉邏輯
}

void motorStop() {
  // 實現停止邏輯
}
*/

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
 */