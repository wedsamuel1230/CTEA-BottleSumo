#include "Motor.h"
#include <Arduino.h>

Motor::Motor(int dir_pin, int pwm_pin)
  :_dir_pin(dir_pin), _pwm_pin(pwm_pin){
    pinMode(_dir_pin, OUTPUT);
    pinMode(_pwm_pin, OUTPUT);

  }


void Motor::set_speed(int speed){
  analogWrite(_pwm_pin, abs(speed));
  if (speed > 0){
    digitalWrite(_dir_pin, HIGH);
  }else if (speed < 0){
    digitalWrite(_dir_pin, LOW);
  }
}

void Motor::stop_motor(){
  analogWrite(_pwm_pin, 0);
}