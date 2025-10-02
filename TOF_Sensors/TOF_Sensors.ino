#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define XSHUT_1 11 // GP11 - Right sensor
#define XSHUT_2 12 // GP12 - Front sensor  
#define XSHUT_3 13 // GP13 - Left sensor

Adafruit_VL53L0X lox1, lox2, lox3;

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

bool init_tof_sensors() {
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);

  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(50); // Faster startup

  // Right sensor - FAST + 1.5m range balance
  digitalWrite(XSHUT_1, HIGH);
  delay(30);
  if (!lox1.begin(0x30, false, &Wire1)) {
    Serial.println("Failed to init RIGHT sensor");
    return false;
  }
  
  // Optimal timing: 50ms = fast + good 1.5m range
  lox1.setMeasurementTimingBudgetMicroSeconds(100000); // 50ms
  lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
  Serial.println("Right sensor: FAST mode (1.5m target)");

  // Front sensor - same config
  digitalWrite(XSHUT_2, HIGH);
  delay(30);
  if (!lox2.begin(0x31, false, &Wire1)) {
    Serial.println("Failed to init FRONT sensor");
    return false;
  }
  lox2.setMeasurementTimingBudgetMicroSeconds(100000);
  lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
  Serial.println("Front sensor: FAST mode (1.5m target)");

  // Left sensor - same config
  digitalWrite(XSHUT_3, HIGH);
  delay(30);
  if (!lox3.begin(0x32, false, &Wire1)) {
    Serial.println("Failed to init LEFT sensor");
    return false;
  }
  lox3.setMeasurementTimingBudgetMicroSeconds(100000);
  lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
  Serial.println("Left sensor: FAST mode (1.5m target)");

  return true;
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

  // Accept status 0-2 for 1.5m range
  reading.right_valid = (data1.RangeStatus <= 2 && data1.RangeMilliMeter < 2000 && data1.RangeMilliMeter > 10);
  reading.right_distance = reading.right_valid ? data1.RangeMilliMeter : 0;

  reading.front_valid = (data2.RangeStatus <= 2 && data2.RangeMilliMeter < 2000 && data2.RangeMilliMeter > 10);
  reading.front_distance = reading.front_valid ? data2.RangeMilliMeter : 0;

  reading.left_valid = (data3.RangeStatus <= 2 && data3.RangeMilliMeter < 2000 && data3.RangeMilliMeter > 10);
  reading.left_distance = reading.left_valid ? data3.RangeMilliMeter : 0;

  return reading;
}

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
  Serial.println();
}

String get_shortest_direction(const ToF_Reading &reading) {
  uint16_t min_distance = 0xFFFF;
  String dir = "CLEAR";
  
  if (reading.right_valid && reading.right_distance < min_distance) {
    min_distance = reading.right_distance;
    dir = "RIGHT";
  }
  if (reading.front_valid && reading.front_distance < min_distance) {
    min_distance = reading.front_distance;
    dir = "FRONT";
  }
  if (reading.left_valid && reading.left_distance < min_distance) {
    min_distance = reading.left_distance;
    dir = "LEFT";
  }
  return dir;
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

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();

  Serial.println("=== FAST MODE (Target: 1.5m) ===");
  if (!init_tof_sensors()) {
    Serial.println("ToF sensor initialization failed!");
    while (1);
  }
  
  Serial.println("ToF sensors ready! Fast readings with 1.5m range");
  scanI2C();
  Serial.println("Starting FAST readings...");
}

void loop() {
  ToF_Reading tof_data = read_tof_sensors();
  print_tof_readings(tof_data);
  
  String shortest_dir = get_shortest_direction(tof_data);
  Serial.print("SHORTEST: ");
  Serial.print(shortest_dir);
  
  // Show the actual shortest distance value
  uint16_t min_dist = 0xFFFF;
  if (tof_data.right_valid && tof_data.right_distance < min_dist) min_dist = tof_data.right_distance;
  if (tof_data.front_valid && tof_data.front_distance < min_dist) min_dist = tof_data.front_distance;
  if (tof_data.left_valid && tof_data.left_distance < min_dist) min_dist = tof_data.left_distance;
  
  if (min_dist < 0xFFFF) {
    Serial.print(" @ ");
    Serial.print(min_dist);
    Serial.println("mm");
  } else {
    Serial.println();
  }

  delay(100); // FAST: ~10 readings per second
}