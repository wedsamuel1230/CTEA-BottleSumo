#pragma once
#include <Arduino.h>
#include <Motor.h>

class MotorController {
public:
  MotorController(uint8_t lpwm, uint8_t ldir, uint8_t rpwm, uint8_t rdir)
    : left(lpwm, ldir), right(rpwm, rdir) {}

  void begin(uint32_t pwmFreqHz = 10000) {
    left.begin(pwmFreqHz);
    right.begin(pwmFreqHz);
  }

  void stopAll() {
    left.stop();
    right.stop();
  }

  // Set motor duty in percent [-100, 100]
  void drive(float leftPct, float rightPct) {
    leftPct = clampPct(leftPct);
    rightPct = clampPct(rightPct);
    left.setDuty(leftPct);
    right.setDuty(rightPct);
  }

private:
  static float clampPct(float v) {
    if (v > 100.0f) return 100.0f;
    if (v < -100.0f) return -100.0f;
    return v;
  }

  Motor left;
  Motor right;
};
