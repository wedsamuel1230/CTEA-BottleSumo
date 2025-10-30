#include "Motor.h"

constexpr uint8_t buttonPin = 28;
bool buttonValue;

Motor motor1(14,15);  // GPIO 11 left notor
Motor motor2(11,12);  // GPIO 10 right motor

unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 100; // 100ms non-blocking interval

void setup() {
  Serial.begin(9600);
  // 兩個馬達使用同一個 slice，頻率相同
  motor1.begin(20000); // 10kHz
  motor2.begin(20000); // 10kHz
  pinMode(buttonPin,INPUT_PULLUP);
  car_stop();
}



void loop() {
  buttonValue = digitalRead(buttonPin);
  Serial.print("Button Value: ");
  Serial.println(buttonValue);
  if (buttonValue == 0){
    car_forward(100);
  }
  else{
    car_stop();
  }
  delay(100);
}

void setup1(){
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop1(){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
void car_stop(){
  motor1.stop();
  motor2.stop();
}

void car_forward(int speed){
  motor1.setDuty(speed);
  motor2.setDuty(speed);
}

void car_backward(int speed){
  motor1.setDuty(-speed);
  motor2.setDuty(-speed);
}