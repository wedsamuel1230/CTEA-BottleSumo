#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define XSHUT_1 11
#define XSHUT_2 12
#define XSHUT_3 13
#define XSHUT_4 14
#define XSHUT_5 15

Adafruit_VL53L0X lox1, lox2, lox3, lox4, lox5;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  
  Serial.println("=== Testing 5 ToF Sensors ===");
  
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_4, OUTPUT);
  pinMode(XSHUT_5, OUTPUT);
  
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  digitalWrite(XSHUT_4, LOW);
  digitalWrite(XSHUT_5, LOW);
  delay(100);
  
  // Sensor 1
  digitalWrite(XSHUT_1, HIGH);
  delay(50);
  if (lox1.begin(0x30, false, &Wire1)) {
    Serial.println("Sensor 1 (RIGHT): OK");
  } else {
    Serial.println("Sensor 1 (RIGHT): FAILED");
  }
  
  // Sensor 2
  digitalWrite(XSHUT_2, HIGH);
  delay(50);
  if (lox2.begin(0x31, false, &Wire1)) {
    Serial.println("Sensor 2 (RIGHT_22): OK");
  } else {
    Serial.println("Sensor 2 (RIGHT_22): FAILED");
  }
  
  // Sensor 3
  digitalWrite(XSHUT_3, HIGH);
  delay(50);
  if (lox3.begin(0x32, false, &Wire1)) {
    Serial.println("Sensor 3 (FRONT): OK");
  } else {
    Serial.println("Sensor 3 (FRONT): FAILED");
  }
  
  // Sensor 4
  digitalWrite(XSHUT_4, HIGH);
  delay(50);
  if (lox4.begin(0x33, false, &Wire1)) {
    Serial.println("Sensor 4 (LEFT_22): OK");
  } else {
    Serial.println("Sensor 4 (LEFT_22): FAILED");
  }
  
  // Sensor 5
  digitalWrite(XSHUT_5, HIGH);
  delay(50);
  if (lox5.begin(0x34, false, &Wire1)) {
    Serial.println("Sensor 5 (LEFT): OK");
  } else {
    Serial.println("Sensor 5 (LEFT): FAILED");
  }
  
  Serial.println("\n=== Starting Readings ===\n");
}

void loop() {
  VL53L0X_RangingMeasurementData_t d1, d2, d3, d4, d5;
  
  lox1.rangingTest(&d1, false);
  lox2.rangingTest(&d2, false);
  lox3.rangingTest(&d3, false);
  lox4.rangingTest(&d4, false);
  lox5.rangingTest(&d5, false);
  
  Serial.print("R1:");
  Serial.print(d1.RangeMilliMeter);
  Serial.print(" R2:");
  Serial.print(d2.RangeMilliMeter);
  Serial.print(" F:");
  Serial.print(d3.RangeMilliMeter);
  Serial.print(" L2:");
  Serial.print(d4.RangeMilliMeter);
  Serial.print(" L5:");
  Serial.println(d5.RangeMilliMeter);
  
  delay(200);
}