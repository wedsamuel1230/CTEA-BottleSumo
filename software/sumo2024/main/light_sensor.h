#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H


class light_sensor{
  public:
    light_sensor(int _pin);
    bool is_inside_table();
  private:
    int _pin;
    //LOW=ture, High=false
};

#endif