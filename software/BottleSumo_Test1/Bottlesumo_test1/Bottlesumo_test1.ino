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
  Serial.println("ToF Sensor Test - Pico W");

  // Setup reset button
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize I2C on Wire1 (GP2=SDA, GP3=SCL)
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  Wire1.setClock(400000);
  Serial.println("I2C Wire1 initialized (400kHz)");
  
  // Scan I2C bus before initialization
  Serial.println("\nBefore initialization:");
  scanI2C();
  
  // Configure and initialize ToF sensors
  tof_array.configure(5, TOF_XSHUT_PINS, TOF_ADDRESSES);
  tof_array.setTiming(50000, 14, 10);  // 50ms timing budget for stable readings
  
  uint8_t online = tof_array.beginAll();
  Serial.printf("\nToF sensors initialized: %d/5\n", online);
  
  // Scan I2C bus after initialization
  Serial.println("\nAfter initialization:");
  scanI2C();
  
  Serial.println("\nStarting sensor readings...\n");
}

void loop() {
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    Serial.println("\n!!! RESET BUTTON PRESSED - Rebooting !!!");
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
      Serial.printf("S%d:---- ", i);
    }
  }
  Serial.printf("(%d/5)\n", validCount);
  
  delay(300);  // Match TimeSliced timing (every ~300ms)
}
