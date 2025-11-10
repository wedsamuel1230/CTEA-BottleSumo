/**
 * Minimal Non-Blocking Sensor Example with 5 ToF Sensors
 * 
 * Quick test for ToFArray and Ads1115Sampler
 * Perfect for initial hardware validation
 * 
 * ToF Sensor Mapping:
 *   Index 0: Right 45° (R45) - XSHUT GP8 - I2C 0x29
 *   Index 1: Right 23° (R23) - XSHUT GP7 - I2C 0x30
 *   Index 2: Middle  0° (M0)  - XSHUT GP6 - I2C 0x31
 *   Index 3: Left  23° (L23)  - XSHUT GP5 - I2C 0x32
 *   Index 4: Left  45° (L45)  - XSHUT GP4 - I2C 0x33
 * 
 * NOTE: Extended initialization timing in ToFArray (_resetDelayMs=100ms, _postResetDelayMs=100ms)
 *       to stabilize sensor startup and prevent I2C conflicts
 *       I2C bus scan added to diagnose connection issues
 */

#include <Wire.h>
#include "ToFArray.h"
#include "Ads1115Sampler.h"

// ToF Configuration: 5 sensors with XSHUT pins
const uint8_t TOF_NUM = 5;
const uint8_t TOF_XSHUT_PINS[TOF_NUM] = {8, 7, 6, 5, 4};  // R45, R23, M0, L23, L45
const uint8_t TOF_I2C_ADDR[TOF_NUM] = {0x30, 0x31, 0x32, 0x33, 0x34};

// Sensor names for debugging
const char* TOF_NAMES[TOF_NUM] = {"R45", "R23", "M0", "L23", "L45"};

// Sensor objects
ToFArray tof(&Wire1,nullptr);
Ads1115Sampler adc;

// Data storage
ToFSample tofData[TOF_NUM];
int16_t adcRaw[4];
float adcVolts[4];

// Timing (milliseconds)
unsigned long lastRead = 0;
const uint32_t READ_INTERVAL = 100;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  Serial.println("Minimal Sensor Test");
  Serial.println("==================");
  
  // Init ADC FIRST (before ToF to allow settling)
  if (adc.begin(0x48, &Wire1, GAIN_ONE, RATE_ADS1115_128SPS)) {
    Serial.println("ADC: Ready");
  } else {
    Serial.println("ADC: FAILED TO INITIALIZE!");
  }
  delay(100);  // Let ADC settle

  // Configure and initialize ToF sensor
  if (tof.configure(TOF_NUM, TOF_XSHUT_PINS, TOF_I2C_ADDR)) {
    Serial.println("ToF: Configured");
  } else {
    Serial.println("ToF: Configuration failed!");
    return;
  }
  
  // Set timing parameters (33ms budget, pre-range=14, final-range=10)
  tof.setTiming(33000, 14, 10);
  
  // Power up and initialize sensor
  uint8_t tofCount = tof.beginAll();
  Serial.printf("ToF: %d/%d sensors online\n", tofCount, TOF_NUM);

  // Report which sensors are online
  Serial.println("\nSensor Status:");
  for (uint8_t i = 0; i < TOF_NUM; i++) {
    Serial.printf("  [%d] %s (GP%d, 0x%02X): %s\n", 
      i, TOF_NAMES[i], TOF_XSHUT_PINS[i], TOF_I2C_ADDR[i],
      tof.isOnline(i) ? "ONLINE" : "OFFLINE");
  }
  
  // I2C Bus Scanner - detect all devices
  Serial.println("\nI2C Bus Scan:");
  Serial.println("-------------");
  uint8_t nDevices = 0;
  
  for (uint8_t address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    uint8_t error = Wire1.endTransmission();
 
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.printf("Scan complete: %d device(s) found\n", nDevices);
  }
 
  delay(2000);  // Wait 2s before starting main loop
  Serial.println();

}

void loop() {
  unsigned long now = millis();
  
  // Read every 100ms
  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;
    
    // Read ToF sensors
    tof.readAll(tofData, 5, 1500, 2);
    
    // Read all 4 ADC channels (blocking, ~32ms @ 128 SPS)
    adc.readAll(adcRaw, adcVolts, 4);
    
    // Print ToF with sensor names and status codes
    Serial.print("| ToF: ");
    for (uint8_t i = 0; i < TOF_NUM; i++) {
      Serial.printf("%s[%d]:%d", TOF_NAMES[i], i, tofData[i].distanceMm);
      if (tofData[i].valid) {
        Serial.print("✓ ");
      } else {
        Serial.printf("✗(s%d) ", tofData[i].status);  // Show error status code
      }
    }
    
    // Print ADC with raw values for debugging
    Serial.print("| ADC: ");
    for (int i = 0; i < 4; i++) {
      Serial.printf("%d:%.2fV(r%d) ", i, adcVolts[i], adcRaw[i]);
    }
    
    Serial.println();
  }
  
  delay(10);
}
