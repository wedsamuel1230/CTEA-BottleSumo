#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// 創建 ADS1115 實例
Adafruit_ADS1115 ads;

// QRE1113 感測器連接到 ADS1115 的通道
const int QRE_CHANNEL_0 = 0;  // A0
const int QRE_CHANNEL_1 = 1;  // A1
const int QRE_CHANNEL_2 = 2;  // A2
const int QRE_CHANNEL_3 = 3;  // A3

// 感測器數值變數
int16_t sensor0_raw, sensor1_raw, sensor2_raw, sensor3_raw;
float sensor0_voltage, sensor1_voltage, sensor2_voltage, sensor3_voltage;

// 閾值設定（可根據實際情況調整）
const float THRESHOLD_VOLTAGE = 1.5;  // 黑線檢測閾值（伏特）

void setup() {
  // 初始化串列通訊
  Serial.begin(115200);
  Serial.println("QRE1113 with ADS1115 初始化中...");
  
  // 初始化 I2C 使用自定義引腳 (GP26=SDA, GP27=SCL)
  Wire.setSDA(26);  // GP26 作為 SDA
  Wire.setSCL(27);  // GP27 作為 SCL
  Wire.begin();
  
  // 初始化 ADS1115
  if (!ads.begin()) {
    Serial.println("無法找到 ADS1115，請檢查接線！");
    while (1);
  }
  
  // 設定 ADS1115 增益（±4.096V 範圍）
  ads.setGain(GAIN_ONE);
  
  Serial.println("ADS1115 初始化完成！");
  Serial.println("開始讀取 QRE1113 感測器數值...");
  Serial.println("格式: CH0 | CH1 | CH2 | CH3 | 狀態");
  Serial.println("----------------------------------------");
}

void loop() {
  // 讀取所有通道的原始數值
  sensor0_raw = ads.readADC_SingleEnded(QRE_CHANNEL_0);
  sensor1_raw = ads.readADC_SingleEnded(QRE_CHANNEL_1);
  sensor2_raw = ads.readADC_SingleEnded(QRE_CHANNEL_2);
  sensor3_raw = ads.readADC_SingleEnded(QRE_CHANNEL_3);
  
  // 轉換為電壓值
  sensor0_voltage = ads.computeVolts(sensor0_raw);
  sensor1_voltage = ads.computeVolts(sensor1_raw);
  sensor2_voltage = ads.computeVolts(sensor2_raw);
  sensor3_voltage = ads.computeVolts(sensor3_raw);
  
  // 顯示數值
  Serial.print("CH0: ");
  Serial.print(sensor0_voltage, 3);
  Serial.print("V | CH1: ");
  Serial.print(sensor1_voltage, 3);
  Serial.print("V | CH2: ");
  Serial.print(sensor2_voltage, 3);
  Serial.print("V | CH3: ");
  Serial.print(sensor3_voltage, 3);
  Serial.print("V | ");
  
  // 檢測黑線狀態
  String status = "狀態: ";
  if (sensor0_voltage < THRESHOLD_VOLTAGE) status += "0";
  else status += "-";
  
  if (sensor1_voltage < THRESHOLD_VOLTAGE) status += "1";
  else status += "-";
  
  if (sensor2_voltage < THRESHOLD_VOLTAGE) status += "2";
  else status += "-";
  
  if (sensor3_voltage < THRESHOLD_VOLTAGE) status += "3";
  else status += "-";
  
  Serial.println(status);
  
  // 延遲 100ms
  delay(100);
}

// 輔助函數：取得特定通道的數值
float getQRESensorVoltage(int channel) {
  int16_t raw_value = ads.readADC_SingleEnded(channel);
  return ads.computeVolts(raw_value);
}

// 輔助函數：檢查特定通道是否檢測到黑線
bool isBlackLineDetected(int channel) {
  float voltage = getQRESensorVoltage(channel);
  return voltage < THRESHOLD_VOLTAGE;
}

// 輔助函數：取得所有感測器的狀態位元組
uint8_t getAllSensorStatus() {
  uint8_t status = 0;
  for (int i = 0; i < 4; i++) {
    if (isBlackLineDetected(i)) {
      status |= (1 << i);
    }
  }
  return status;
}
