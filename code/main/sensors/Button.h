#ifndef BUTTON_H
#define BUTTON_H
// =====================

#include <Arduino.h>

class Button {
  public:
  int pin;
  bool state;

  bool getState(){
    int button_value = digitalRead(pin);

    if (button_value == HIGH){
      state = true;
    } else {
      state = false;
    }
    return state;
  }

  void initialize(){
    pinMode(pin,INPUT_PULLDOWN);
  }
};

#endif