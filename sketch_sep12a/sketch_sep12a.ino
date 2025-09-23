#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Change to your chosen GPIO
#define XSHUT_1 11 // GP11
#define XSHUT_2 12 // GP12
#define XSHUT_3 13 // GP13

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();

  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);

  // Turn sensor OFF
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(100);

  // Change the TOF sensor Address

  //Sensor 1

  digitalWrite(XSHUT_1, HIGH);
  delay(100);
  lox1.begin(0x70,false,&Wire1);
  delay(100);
  scanI2C();
  delay(100);
  digitalWrite(XSHUT_1, LOW);
  delay(100);

  //sensor 2
  digitalWrite(XSHUT_2, HIGH);
  delay(100);
  lox2.begin(0x71,false,&Wire1);
  delay(100);
  scanI2C();
  delay(100);
  digitalWrite(XSHUT_2, LOW);
  delay(100);

  //sensor3
  digitalWrite(XSHUT_2, HIGH);
  delay(100);
  lox3.begin(0x72,false,&Wire1);
  delay(100);
  scanI2C();
  delay(100);
  digitalWrite(XSHUT_3, LOW);
  delay(100);
  
  // Turn sensor back ON for normal operation
  digitalWrite(XSHUT_1, HIGH);
  digitalWrite(XSHUT_2, HIGH);
  digitalWrite(XSHUT_3, HIGH);
}

void loop() {
  // do nothing
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