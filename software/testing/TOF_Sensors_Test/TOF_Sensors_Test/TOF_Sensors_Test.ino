#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define XSHUT_1 11
#define XSHUT_2 12
#define XSHUT_3 13
#define XSHUT_4 14
#define XSHUT_5 15

Adafruit_VL53L0X lox1, lox2, lox3, lox4, lox5;
bool sensor_active[5] = {false, false, false, false, false};

void scanI2C() {
  Serial.println("\n--- I2C Scan ---");
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    if (error == 0) {
      Serial.print("Found: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  Serial.print("Total: "); Serial.println(nDevices);
  Serial.println("----------------\n");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  
  Serial.println("\n=== 5-Sensor ToF Debug Init ===\n");
  
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_4, OUTPUT);
  pinMode(XSHUT_5, OUTPUT);
  
  // All OFF
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  digitalWrite(XSHUT_4, LOW);
  digitalWrite(XSHUT_5, LOW);
  delay(200);
  
  Serial.println("Initial I2C scan (all sensors OFF):");
  scanI2C();
  
  // Sensor 1
  Serial.println("Enabling Sensor 1...");
  digitalWrite(XSHUT_1, HIGH);
  delay(100);
  scanI2C();
  if (lox1.begin(0x30, false, &Wire1)) {
    lox1.setMeasurementTimingBudgetMicroSeconds(50000);
    lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    Serial.println("✓ Sensor 1: OK\n");
    sensor_active[0] = true;
  } else {
    Serial.println("✗ Sensor 1: FAILED\n");
  }
  delay(50);
  
  // Sensor 2
  Serial.println("Enabling Sensor 2...");
  digitalWrite(XSHUT_2, HIGH);
  delay(100);
  scanI2C();
  if (lox2.begin(0x31, false, &Wire1)) {
    lox2.setMeasurementTimingBudgetMicroSeconds(50000);
    lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    Serial.println("✓ Sensor 2: OK\n");
    sensor_active[1] = true;
  } else {
    Serial.println("✗ Sensor 2: FAILED\n");
  }
  delay(50);
  
  // Sensor 3
  Serial.println("Enabling Sensor 3...");
  digitalWrite(XSHUT_3, HIGH);
  delay(100);
  scanI2C();
  if (lox3.begin(0x32, false, &Wire1)) {
    lox3.setMeasurementTimingBudgetMicroSeconds(50000);
    lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    Serial.println("✓ Sensor 3: OK\n");
    sensor_active[2] = true;
  } else {
    Serial.println("✗ Sensor 3: FAILED - CONTINUING ANYWAY\n");
  }
  delay(50);
  
  // Sensor 4
  Serial.println("Enabling Sensor 4...");
  digitalWrite(XSHUT_4, HIGH);
  delay(100);
  scanI2C();
  if (lox4.begin(0x33, false, &Wire1)) {
    lox4.setMeasurementTimingBudgetMicroSeconds(50000);
    lox4.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    lox4.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    Serial.println("✓ Sensor 4: OK\n");
    sensor_active[3] = true;
  } else {
    Serial.println("✗ Sensor 4: FAILED\n");
  }
  delay(50);
  
  // Sensor 5
  Serial.println("Enabling Sensor 5...");
  digitalWrite(XSHUT_5, HIGH);
  delay(100);
  scanI2C();
  if (lox5.begin(0x34, false, &Wire1)) {
    lox5.setMeasurementTimingBudgetMicroSeconds(50000);
    lox5.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    lox5.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    Serial.println("✓ Sensor 5: OK\n");
    sensor_active[4] = true;
  } else {
    Serial.println("✗ Sensor 5: FAILED\n");
  }
  
  Serial.println("=== Starting Readings ===\n");
}

void loop() {
  VL53L0X_RangingMeasurementData_t d1, d2, d3, d4, d5;
  
  if (sensor_active[0]) {
    lox1.rangingTest(&d1, false);
    Serial.print("S1:"); Serial.print(d1.RangeMilliMeter); Serial.print(" ");
  }
  if (sensor_active[1]) {
    lox2.rangingTest(&d2, false);
    Serial.print("S2:"); Serial.print(d2.RangeMilliMeter); Serial.print(" ");
  }
  if (sensor_active[2]) {
    lox3.rangingTest(&d3, false);
    Serial.print("S3:"); Serial.print(d3.RangeMilliMeter); Serial.print(" ");
  }
  if (sensor_active[3]) {
    lox4.rangingTest(&d4, false);
    Serial.print("S4:"); Serial.print(d4.RangeMilliMeter); Serial.print(" ");
  }
  if (sensor_active[4]) {
    lox5.rangingTest(&d5, false);
    Serial.print("S5:"); Serial.println(d5.RangeMilliMeter);
  } else {
    Serial.println();
  }
  
  delay(200);
}