/**
 * ADS1115 Sampler Example
 * Reads all four single-ended channels using Ads1115Sampler and prints raw + voltage.
 * 
 * Wiring (default I2C):
 *   - SDA: GP0 (or board SDA)
 *   - SCL: GP1 (or board SCL)
 *   - ADDR pin floating -> I2C address 0x48 (default)
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Ads1115Sampler.h>

// Choose your I2C bus (default Wire)
TwoWire& bus = Wire;
constexpr uint8_t ADS_ADDR = 0x48;
constexpr adsGain_t ADS_GAIN = GAIN_ONE;   // ±4.096V
constexpr uint8_t ADS_RATE = 128;          // 128 samples per second (~32ms full read)

Ads1115Sampler sampler;
int16_t raw[4];
float volts[4];

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ADS1115 Sampler starting...");

  bus.begin();
  bool ok = sampler.begin(ADS_ADDR, &bus, ADS_GAIN, ADS_RATE);
  if (!ok) {
    Serial.println("Failed to init ADS1115. Check wiring/address.");
    while (1) { delay(100); }
  }
  Serial.println("ADS1115 ready.");
}

void loop() {
  sampler.readAll(raw, volts, 4);
  Serial.print("RAW:");
  for (int i = 0; i < 4; ++i) {
    Serial.print(" ");
    Serial.print(raw[i]);
  }
  Serial.print(" | VOLTS:");
  for (int i = 0; i < 4; ++i) {
    Serial.print(" ");
    Serial.print(volts[i], 4);
  }
  Serial.println();
  delay(500);
}
