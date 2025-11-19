#ifndef MOTOR_H

#define MOTOR_H

class Motor{
  public:
    Motor(int dir_pin, int pwm_pin);
    void set_speed(int speed);
    void stop_motor();

  private:
    int _dir_pin;
    int _pwm_pin;
};

#endif