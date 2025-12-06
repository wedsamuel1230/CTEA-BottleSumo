/**
 * ToFArray Basic Example (VL53L0X)
 * Manages multiple VL53L0X sensors with XSHUT control to assign unique I2C addresses.
 *
 * Wiring (example for 2 sensors):
 *   - Shared SDA / SCL on default I2C bus
 *   - Sensor 0 XSHUT -> GP6
 *   - Sensor 1 XSHUT -> GP7
 *   - Power: 3V3 and GND
 *
 * Addresses chosen here: 0x30, 0x31
 */

#include <Wire.h>
#include <ToFArray.h>

// Optional bus mutex for RP2040 (pico-sdk)
#if defined(ARDUINO_ARCH_RP2040)
#include <pico/mutex.h>
mutex_t busMutex;
#endif

constexpr uint8_t SENSOR_COUNT = 2;
constexpr uint8_t XSHUT_PINS[SENSOR_COUNT] = {6, 7};
constexpr uint8_t I2C_ADDRS[SENSOR_COUNT] = {0x30, 0x31};

ToFArray tof(&Wire,
#if defined(ARDUINO_ARCH_RP2040)
             &busMutex
#else
             nullptr
#endif
);

ToFSample samples[SENSOR_COUNT];

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ToFArray basic start");

#if defined(ARDUINO_ARCH_RP2040)
  mutex_init(&busMutex);
#endif
  Wire.begin();

  if (!tof.configure(SENSOR_COUNT, XSHUT_PINS, I2C_ADDRS)) {
    Serial.println("Configure failed (bad count)");
    while (1) { delay(100); }
  }

  tof.setTiming(/*budget_us=*/33000, /*preRange=*/14, /*finalRange=*/10);
  uint8_t ok = tof.beginAll();
  Serial.print("Sensors online: ");
  Serial.println(ok);
  if (ok == 0) {
    Serial.println("No sensors detected. Check wiring and XSHUT pins.");
    while (1) { delay(100); }
  }
}

void loop() {
  tof.readAll(samples, /*minMm=*/30, /*maxMm=*/1500, /*maxStatus=*/2);

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print("S"); Serial.print(i);
    Serial.print(samples[i].valid ? ": " : " (invalid): ");
    Serial.print(samples[i].distanceMm);
    Serial.print(" mm (status=");
    Serial.print(samples[i].status);
    Serial.print(")  ");
  }
  Serial.println();
  delay(100);
}
