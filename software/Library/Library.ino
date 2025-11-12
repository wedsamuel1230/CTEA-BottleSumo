/**
 * CTEA Bottle Sumo Robot - Library Example
 * 
 * Demonstrates the Motor and Car library usage for RP2040 (Raspberry Pi Pico)
 * 
 * Hardware Configuration:
 * - Left Motor:  PWM=GP2, DIR=GP3
 * - Right Motor: PWM=GP4, DIR=GP5
 * - PWM Frequency: 1000 Hz (default)
 */

#include "Car.h"

// Pin definitions (adjust for your hardware)
const uint8_t LEFT_PWM_PIN  = 2;
const uint8_t LEFT_DIR_PIN  = 3;
const uint8_t RIGHT_PWM_PIN = 4;
const uint8_t RIGHT_DIR_PIN = 5;

// PWM frequency for motor control
// Available ranges (from Motor.h):
//   - Minimum: ~1.9 kHz (max resolution, TOP=65535)
//   - Optimal for motors: 20 kHz (Motor::PWM_FREQ_MOTOR_OPTIMAL) - silent operation
//   - Maximum recommended: ~488 kHz (TOP=256, 0.4% resolution steps)
//   - Maximum practical: 1.25 MHz (TOP=100, 1% resolution - still usable)
//   - Maximum absolute: 62.5 MHz (TOP=1, only 0%/100% - NOT USEFUL for motors)
//
// Trade-off: Higher frequency = faster switching, BUT lower duty cycle resolution
// For smooth motor control, use 10-50 kHz. For high-speed digital signals, up to 1.25 MHz.
const uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz (recommended for silent motor operation)
// Alternative high-performance option:
// const uint32_t MOTOR_PWM_FREQ = Motor::PWM_FREQ_MAX_RECOMMENDED;  // ~488 kHz

// Create car instance
Car robot;

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial to stabilize
  
  Serial.println("CTEA Bottle Sumo - Initializing...");
  
  // Initialize motors
  bool success = robot.initializeMotors(
    LEFT_PWM_PIN, LEFT_DIR_PIN,
    RIGHT_PWM_PIN, RIGHT_DIR_PIN,
    MOTOR_PWM_FREQ
  );
  
  if (success) {
    Serial.println("✓ Motors initialized successfully");
  } else {
    Serial.println("✗ Motor initialization FAILED - check pin configuration");
    while(1) { delay(100); }  // Halt on error
  }
  
  Serial.println("Starting movement sequence in 2 seconds...");
  delay(2000);
}

void loop() {
  // Movement demonstration sequence
  Serial.println("Forward 50%");
  robot.forward(50);
  delay(2000);
  
  Serial.println("Backward 50%");
  robot.backward(50);
  delay(2000);
  
  Serial.println("Turn Left 70%");
  robot.turnLeft(70);
  delay(1000);
  
  Serial.println("Turn Right 70%");
  robot.turnRight(70);
  delay(1000);
  
  Serial.println("Stop");
  robot.stop();
  delay(2000);
  
  // Advanced: Manual differential drive
  Serial.println("Differential: L=30% R=60%");
  robot.setMotors(30, 60);
  delay(1500);
  
  robot.stop();
  delay(3000);
}

