#include "button.h"
#include <Arduino.h>

Button::Button(int _pin)
  :_pin(_pin){
    pinMode(_pin, INPUT_PULLUP);
  }


bool Button::is_button_pressed(){
  if (digitalRead(_pin) == LOW) {
    return true;
  }
  return false;
}