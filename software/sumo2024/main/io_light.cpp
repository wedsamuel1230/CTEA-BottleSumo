#include "io_light.h"

//Constructor
io_light::io_light(uint8_t light_pin)
  :light_pin(light_pin){
    pinMode(light_pin, OUTPUT);
  }

//functions
void io_light::flash_light(int loop_times){
  for (int i = 0; i < loop_times; i++){
    digitalWrite(light_pin, HIGH);
    delay(1000);
    digitalWrite(light_pin, LOW);
    delay(1000);
  }
}

void io_light::switch_light(int state){
  digitalWrite(light_pin, state);
}