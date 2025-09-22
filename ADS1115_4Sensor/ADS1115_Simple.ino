/*
 * ADS1115 Simple Example for Raspberry Pi Pico
 * 
 * This is a simplified version that demonstrates basic usage
 * of the ADS1115 with 4 sensors, following the patterns
 * found in the repository's existing sensor code.
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

// Constants matching repository patterns
constexpr uint8_t NUM_SENS = 4U;
constexpr float VREF = 4.096F;  // ADS1115 reference voltage
const uint8_t sensChannels[NUM_SENS] = {0, 1, 2, 3};
const uint8_t sensMask[NUM_SENS] = {0b0001, 0b0010, 0b0100, 0b1000};

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }
  
  // I2C setup (matching repository pattern)
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  
  // Initialize ADS1115
  if (!ads.begin(0x48, &Wire1)) {
    Serial.println(F("ADS1115 initialization failed!"));
    while (1) { delay(1000); }
  }
  
  ads.setGain(GAIN_ONE);  // ±4.096V
  Serial.println(F("ADS1115 4-Sensor initialized"));
}

void loop() {
  checkSensors(2.5);  // 2.5V threshold
  delay(200);
}

int checkSensors(float thresholdV) {
  uint8_t result = 0;
  
  for (uint8_t i = 0; i < NUM_SENS; ++i) {
    int16_t adcValue = ads.readADC_SingleEnded(sensChannels[i]);
    float volts = ads.computeVolts(adcValue);
    
    if (volts >= thresholdV) {
      result |= sensMask[i];
    }
    
    Serial.print(F("Voltage("));
    Serial.print(i + 1);
    Serial.print(F("): "));
    Serial.print(volts, 2);
    Serial.print(F("V\n"));
  }
  
  Serial.print(F("Result (DEC): "));
  Serial.println(result);
  return result;
}