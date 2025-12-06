/*
* Motor Test Example
* Tests Motor and Car library functionality on RP2040 (Raspberry Pi Pico)
* 
* Wiring (for two motors):
*   - Left Motor:  PWM=GP11, DIR=GP12
*   - Right Motor: PWM=GP14, DIR=GP15
*   - Power: 12V and GND
*/

#include <Car.h>

// Pin mapping (adjust to your wiring)
constexpr uint8_t LEFT_MOTOR_PWM_PIN  = 11;
constexpr uint8_t LEFT_MOTOR_DIR_PIN  = 12;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 14;
constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 15;

// Use the library constant for optimal, silent operation (20 kHz)
constexpr uint32_t MOTOR_FREQ = Motor::PWM_FREQ_MOTOR_OPTIMAL;

Car car;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  bool ok = car.initializeMotors(
    LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN,
    RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN,
    MOTOR_FREQ
  );

  if (!ok || !car.isInitialized()) {
    Serial.println("[motor_test] Failed to initialize motors");
    while (1) { delay(100); }
  }

  Serial.println("[motor_test] Motors initialized @ 20 kHz");
  car.stop();
  delay(500);
}

void loop() {
  // Simple motion script
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Forward 40%");
  car.forward(40);
  delay(1500);

  Serial.println("Backward 40%");
  car.backward(40);
  delay(1500);

  Serial.println("Turn Left 60%");
  car.turnLeft(60);
  delay(800);

  Serial.println("Turn Right 60%");
  car.turnRight(60);
  delay(800);

  Serial.println("Stop");
  car.stop();
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
}