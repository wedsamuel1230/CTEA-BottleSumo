/**
 * Minimal Non-Blocking Sensor Example with 1 ToF Sensor
 * 
 * Quick test for ToFArray and Ads1115Sampler
 * Perfect for initial hardware validation
 * 
 * ToF Sensor: XSHUT pin = GP8 | I2C address = 0x29
 * 
 * NOTE: Extended initialization timing in ToFArray (_resetDelayMs=100ms, _postResetDelayMs=50ms)
 *       to stabilize sensor startup and prevent I2C conflicts
 */

#include <Wire.h>
#include "ToFArray.h"
#include "Ads1115Sampler.h"

// ToF Configuration: 1 sensor with XSHUT pin
const uint8_t TOF_NUM = 1;
const uint8_t TOF_XSHUT_PINS[TOF_NUM] = {8};
const uint8_t TOF_I2C_ADDR[TOF_NUM] = {0x29};

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

// Track which ADC channel we're currently reading
uint8_t currentAdcChannel = 0;

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
  
  Serial.println();
}

void loop() {
  unsigned long now = millis();
  
  // Read every 100ms
  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;
    
    // Read ToF sensor
    tof.readAll(tofData, 30, 1500, 2);
    
    // Non-blocking ADC read: start conversion on each channel in sequence
    adc.startConversion(currentAdcChannel);
    
    // Poll for completion (20ms timeout at 128 SPS for safety)
    if (adc.poll()) {
      adcRaw[currentAdcChannel] = adc.getLastResult();
      adcVolts[currentAdcChannel] = adc.getLastResultVolts();
      
      // Move to next channel (0-3, wrap around)
      currentAdcChannel = (currentAdcChannel + 1) % 4;
    }
    
    // Print ToF
    Serial.print("| ToF: ");
    Serial.printf("%dmm(%s) ", tofData[0].distanceMm, tofData[0].valid ? "ok" : "xx");
    
    // Print ADC
    Serial.print("| ADC: ");
    Serial.printf("%d:%.2fV ", 0, adcVolts[0]);
    Serial.printf("%d:%.2fV ", 1, adcVolts[1]);
    Serial.printf("%d:%.2fV ", 2, adcVolts[2]);
    Serial.printf("%d:%.2fV ", 3, adcVolts[3]);
    
    Serial.println();
  }
  
  delay(10);
}
