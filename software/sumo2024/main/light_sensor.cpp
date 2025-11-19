#include "light_sensor.h"
#include <Arduino.h>

//Constructor
light_sensor::light_sensor(int pin)
  :_pin(pin){
    pinMode(_pin, INPUT);
  }

//functions
bool light_sensor::is_inside_table(){
  int state = digitalRead(_pin);
  Serial.println(state);
  if (state == HIGH){
    return false;
  }
  return true;
}