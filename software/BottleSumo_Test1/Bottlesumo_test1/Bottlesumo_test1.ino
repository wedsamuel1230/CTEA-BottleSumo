#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include "ToFArray.h"

// Pin definitions - XSHUT pins are GP3, GP4, GP5, GP6, GP7, GP8
const uint8_t TOF_XSHUT_PINS[5] = {8, 7, 6, 5, 4};  // GP8, GP7, GP6, GP5, GP4
const uint8_t TOF_ADDRESSES[5] = {0x30, 0x31, 0x32, 0x33, 0x34};
const uint8_t RESET_BUTTON_PIN = 18;

ToFArray tof_array(&Wire1, nullptr);

void scanI2C() {
  byte error, address;
  int nDevices = 0;
  
  Serial.print("I2C devices: ");
  for(address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    
    if (error == 0) {
      Serial.printf("0x%02X ", address);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.print("None");
  }
  Serial.printf(" (%d found)\n", nDevices);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Setup reset button
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize I2C
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  Wire1.setClock(400000);
  
  // Configure and initialize ToF sensors
  tof_array.configure(5, TOF_XSHUT_PINS, TOF_ADDRESSES);
  tof_array.setTiming(100000, 14, 10);  // 100ms timing budget (was 300ms)
  delay(100);
  uint8_t online = tof_array.beginAll();
  delay(100);
  Serial.printf("ToF sensors initialized: %d/5\n", online);
  
  scanI2C();
  Serial.println();
}

void loop() {
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    Serial.println("RESET!");
    delay(1000);
    rp2040.reboot();
  }
  
  ToFSample samples[5];
  tof_array.readAll(samples, 30, 1500, 2);
  
  int validCount = 0;
  for (int i = 0; i < 5; i++) {
    if (samples[i].valid) {
      Serial.printf("S%d:%4dmm ", i, samples[i].distanceMm);
      validCount++;
    } else {
      // Show status code to help debug
      Serial.printf("S%d:ERR(0x%02X) ", i, samples[i].status);
    }
  }
  Serial.printf("(%d/5)\n", validCount);

  scanI2C();
  Serial.println();
  
  delay(500);
}
