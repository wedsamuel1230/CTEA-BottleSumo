#include "Car.h"

constexpr uint8_t LEFT_MOTOR_PWM_PIN = 11;
constexpr uint8_t LEFT_MOTOR_DIR_PIN = 12;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 14;
constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 15;
constexpr uint32_t MOTOR_FREQ = 20000; // 20 kHz

void setup() {
  Serial.begin(9600);
  //make the pins outputs 
  pinMode(LeftMotorPWM, OUTPUT);
	pinMode(RightMotorPWM, OUTPUT);
  pinMode(btn, INPUT_PULLUP);
}

void move_straight(uint8_t speed_for_Motor1 = 0,uint8_t speed_for_motor2 = 0){
	analogWrite(LeftMotorPWM,speed_for_Motor1);
	analogWrite(RightMotorPWM,speed_for_motor2);
}
void loop() {
  if (digitalRead(btn) == LOW){
    move_straight(255,255);
  }
  else {
    move_straight();
  }
  delay(50);
  /*
	move_straight(0,0);//full speed
	delay(2500);
	move_straight(127,127);//half speed
	delay(2500);
	move_straight(255,255);//stop
	delay(2500);
  */
}
