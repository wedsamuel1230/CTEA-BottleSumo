/**
 * Minimal Non-Blocking Sensor Example
 * 
 * Quick test for ToFArray and Ads1115Sampler
 * Perfect for initial hardware validation
 */

#include <Wire.h>
#include "ToFArray.h"
#include "Ads1115Sampler.h"

// Sensor objects
ToFArray tof(&Wire, nullptr);
Ads1115Sampler adc;

// Data storage
ToFSample tofData[3];
int16_t adcRaw[4];
float adcVolts[4];

// Timing (milliseconds)
unsigned long lastRead = 0;
const uint32_t READ_INTERVAL = 100;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Minimal Sensor Test");
  Serial.println("==================");
  
  // Init I2C
  Wire.begin();
  
  // Init ToF (3 sensors)
  uint8_t xshut[] = {16, 17, 18};
  uint8_t addr[] = {0x30, 0x31, 0x32};
  if (tof.configure(3, xshut, addr)) {
    Serial.printf("ToF: %d sensors online\n", tof.beginAll());
  }
  
  // Init ADC
  if (adc.begin(0x48, &Wire, GAIN_TWOTHIRDS, RATE_ADS1115_128SPS)) {
    Serial.println("ADC: Ready");
  }
  
  Serial.println();
}

void loop() {
  unsigned long now = millis();
  
  // Read every 100ms
  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;
    
    // Read sensors
    tof.readAll(tofData);
    adc.readAll(adcRaw, adcVolts, 4);
    
    // Print ToF
    Serial.print("ToF: ");
    for (int i = 0; i < 3; i++) {
      if (tofData[i].valid) {
        Serial.printf("%d:%4dmm ", i, tofData[i].distanceMm);
      } else {
        Serial.printf("%d:---- ", i);
      }
    }
    
    // Print ADC
    Serial.print("| ADC: ");
    for (int i = 0; i < 4; i++) {
      Serial.printf("%d:%.2fV ", i, adcVolts[i]);
    }
    
    Serial.println();
  }
  
  delay(1);
}
