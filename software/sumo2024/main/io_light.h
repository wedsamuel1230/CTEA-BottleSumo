#ifndef IO_LIGHT_H
#define IO_LIGHT_H

#include <Arduino.h>

// LED_BUILTIN

class io_light{
  public:
    io_light(uint8_t light_pin);
    void flash_light(int loop_times);
    void switch_light(int state);
  
  private:
    uint8_t light_pin;

};

#endif