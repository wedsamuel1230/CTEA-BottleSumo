#include <Wire.h>
#include <VL53L1X.h>

// I2C
TwoWire i2c(i2c0, 26, 27); // i2c0, SDA=GP26, SCL=GP27

// XSHUT pins for sensors
const int xshutPins[] = {13, 12, 11};
const int numSensors = 3;
VL53L1X sensors[numSensors];
const uint8_t newAddresses[] = {0x30, 0x32, 0x34};

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  i2c.begin();

  // Initialize sensors
  for (int i = 0; i < numSensors; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  for (int i = 0; i < numSensors; i++) {
    digitalWrite(xshutPins[i], HIGH);
    delay(50);

    sensors[i].setBus(&i2c);
    sensors[i].setAddress(0x29);
    sensors[i].init();
    sensors[i].setDistanceMode(VL53L1X::Short);
    sensors[i].setTimingBudget(50); // Use setTimingBudget instead of setTimingBudgetInMicroSeconds
    sensors[i].startContinuous(50);
    sensors[i].setAddress(newAddresses[i]);
    delay(10);
  }

  // Keep XSHUT high
  for (int i = 0; i < numSensors; i++) {
    digitalWrite(xshutPins[i], HIGH);
  }
}

void loop() {
  int distances[numSensors];

  // Read distances
  for (int i = 0; i < numSensors; i++) {
    distances[i] = sensors[i].read();
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(distances[i]);
    Serial.println(" mm");
  }

  delay(500); // Delay for readability in Serial Monitor
}
