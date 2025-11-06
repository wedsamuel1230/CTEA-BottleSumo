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
ToFArray tof(&Wire1, nullptr);
Ads1115Sampler adc;

// Data storage
ToFSample tofData[3];
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
  // Init ADC
  if (adc.begin(0x48, &Wire1, GAIN_ONE, RATE_ADS1115_128SPS)) {
    Serial.println("ADC: Ready");
  }
  
  Serial.println();
}

void loop() {
  unsigned long now = millis();
  
  // Read every 100ms
  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;
    
    // Non-blocking ADC read: start conversion on each channel in sequence
    adc.startConversion(currentAdcChannel);
    
    // Poll for completion (typically completes within 8-16ms at 128 SPS)
    if (adc.poll()) {
      adcRaw[currentAdcChannel] = adc.getLastResult();
      adcVolts[currentAdcChannel] = adc.getLastResultVolts();
      
      // Move to next channel (0-3, wrap around)
      currentAdcChannel = (currentAdcChannel + 1) % 4;
    }
    
    // Print ADC
    Serial.print("| ADC: ");
    for (int i = 0; i < 4; i++) {
      Serial.printf("%d:%.2fV ", i, adcVolts[i]);
    }
    
    Serial.println();
  }
  
  delay(10);
}
