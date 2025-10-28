#include "Motor.h"

Motor motor1(14,15);  // GPIO 11 left notor
Motor motor2(7,8);  // GPIO 10 right motor

unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 100; // 100ms non-blocking interval

void setup() {
  Serial.begin(9600);
  // 兩個馬達使用同一個 slice，頻率相同
  motor1.begin(20000); // 10kHz
  motor2.begin(20000); // 10kHz
  pinMode(18,INPUT_PULLUP);
  car_stop();
}

void loop() {
  Serial.println(digitalRead(18));
  if (digitalRead(18) == 0){
    car_forward(100);
  }
  else{
    car_stop();
  }
  delay(100);
}

void car_stop(){
  motor1.stop();
  motor2.stop();
}

void car_forward(int speed){
  motor1.setDuty(speed);
  motor2.setDuty(-speed);
}

void car_backward(int speed){
  motor1.setDuty(-speed);
  motor2.setDuty(speed);
}