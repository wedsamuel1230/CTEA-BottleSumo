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
#include <pico/multicore.h>

// Pin definitions (adjust for your hardware)
const uint8_t LEFT_PWM_PIN  = 14;
const uint8_t LEFT_DIR_PIN  = 15;
const uint8_t RIGHT_PWM_PIN = 11;
const uint8_t RIGHT_DIR_PIN = 12;
const uint8_t buttonPIN = 28;
const uint8_t LED_PIN = 25;  // Built-in LED on Raspberry Pi Pico

int buttonValue;
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
//const uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz (recommended for silent motor operation)
// Alternative high-performance option:
const uint32_t MOTOR_PWM_FREQ = Motor::PWM_FREQ_MAX_RECOMMENDED;  // ~488 kHz

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
  robot.stop();
  delay(2000);
  
  // Set button pin as input
  pinMode(buttonPIN, INPUT_PULLUP);
}

void loop() {
  // Read button state
  buttonValue = digitalRead(buttonPIN);
  Serial.print("buttonValue");
  Serial.println(buttonValue);
  // If button is NOT pressed (HIGH), move forward at full speed
  if (buttonValue == LOW) {
    robot.forward(100);
    Serial.println("Button not pressed - Forward 100%");
  } 
  else {
    // Button pressed (LOW), stop
    robot.stop();
  }
  
  delay(50);  // Small delay for debouncing
}

// Core 1 - LED Blink
void setup1() {
  // Initialize LED pin on core 1
  pinMode(LED_PIN, OUTPUT);
  Serial.println("✓ Core 1 LED initialized");
}

void loop1() {
  // Blink LED on core 1 (500ms on, 500ms off)
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}

