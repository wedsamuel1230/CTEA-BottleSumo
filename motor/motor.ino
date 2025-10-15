#include "Motor.h"

Motor motor1(11, 12);  // GPIO 11 left notor
Motor motor2(20, 19);  // GPIO 10 right motor

void setup() {
  Serial.begin(9600);
  delay(1000);
  // 兩個馬達使用同一個 slice，頻率相同
  motor1.begin(20000); // 10kHz
  motor2.begin(20000); // 10kHz
}

void loop() {
  // 你的主程式邏輯...
  delay(3000);
  car_forward(90);
  delay(3000);
  car_stop();
  delay(10000);
  car_backward(20);
  
}


//--- other function ---//
//--- car controlling
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

void car_rotate_turn_left(int speed){ //自轉
  motor1.setDuty(-speed);
  motor2.setDuty(-speed);
}

void car_rotate_turn_right(int speed){ //自轉
  motor1.setDuty(speed);
  motor2.setDuty(speed);
}

void car_revolve_turn_left(int speed){
  motor1.stop();
  motor2.setDuty(-speed);
}

void car_revolve_turn_right(int speed){
  motor1.setDuty(speed);
  motor2.stop();
  
}