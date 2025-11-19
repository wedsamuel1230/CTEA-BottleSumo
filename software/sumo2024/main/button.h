#ifndef BUTTON_H

#define BUTTON_H

class Button{
  public:
    Button(int _pin);
    bool is_button_pressed();

  private:
    int _pin;
};

#endif