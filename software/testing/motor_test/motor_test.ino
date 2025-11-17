#include "Car.h"

constexpr uint8_t LEFT_MOTOR_PWM_PIN = 11;
constexpr uint8_t LEFT_MOTOR_DIR_PIN = 12;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 14;
constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 15;
constexpr uint32_t MOTOR_FREQ = 20000; // 20 kHz

Car car;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  car.initializeMotors(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN,
                       RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN,
                       MOTOR_FREQ);
  if (!car.isInitialized()) {
      Serial.println("Failed to initialize motors");
      while (1);
  }
  Serial.println("Motors initialized successfully");
  car.stop();
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  car.stop();
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  car.forward(0.0F);
  delay(2000);
}