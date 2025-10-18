#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
  Motor(uint pwm_pin, uint dir_pin);
  void setDuty(float speed); // duty -100 ~ 100
  void stop();
  void begin(uint32_t freq);

private:
  uint _pwm_pin, _dir_pin;
  uint _slice;
  uint _channel;
  uint16_t _top;
};

#endif
