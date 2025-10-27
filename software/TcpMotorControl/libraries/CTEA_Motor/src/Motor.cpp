#include <sys/_stdint.h>
#include <Arduino.h>
#include "hardware/pwm.h"

#include "Motor.h"

Motor::Motor(uint pwm_pin, uint dir_pin) : _pwm_pin(pwm_pin), _dir_pin(dir_pin) {}

void Motor::begin(uint32_t freq) {
  // 設定 GPIO 為 PWM 功能
  gpio_set_function(_pwm_pin, GPIO_FUNC_PWM);
  pinMode(_dir_pin, OUTPUT);

  // 找出該 pin 所屬 slice 與 channel
  _slice = pwm_gpio_to_slice_num(_pwm_pin);
  _channel = pwm_gpio_to_channel(_pwm_pin);

  // 計算 wrap 值（TOP）
  // 預設 clock 125 MHz
  uint32_t clock = 125000000;
  _top = clock / freq - 1;

  pwm_set_wrap(_slice, _top);

  // 開始輸出
  pwm_set_enabled(_slice, true);
}

void Motor::setDuty(float speed) {
  float duty;
  uint8_t dir;
  // 設置方向及duty
  if (speed > 0){ // 正方向
    duty = speed * 0.01;
    dir = 1;
  }else if (speed < 0){ // 反方向
    duty = speed * -0.01;
    dir = 0;
  }
  if (duty > 1) duty = 1; // No higher than 100%

  Serial.print("Motor dir:");
  Serial.println(dir);

  if (dir == 0){
    digitalWrite(_dir_pin, LOW);
  }else if (dir == 1){
    digitalWrite(_dir_pin, HIGH);
  }
  uint16_t level = (uint16_t)(duty * _top);
  pwm_set_chan_level(_slice, _channel, level);


}

void Motor::stop() {
  Serial.println("stopped");
  float duty = 0;

  uint16_t level = (uint16_t)(duty * _top);
  pwm_set_chan_level(_slice, _channel, level);
  digitalWrite(_dir_pin, LOW);
}
