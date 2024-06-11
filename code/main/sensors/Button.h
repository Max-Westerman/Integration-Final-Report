/**
 * @file Button.h
 * @brief Defines the Button class for handling button input on an Arduino.
 * 
 * This file contains the definition of the Button class, which is used to manage
 * the state and initialization of a button connected to an Arduino pin.
 * 
 * Created by: Max Westerman
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

class Button {
  public:
    int pin;     // Pin number to which the button is connected
    bool state;  // Current state of the button (true for pressed, false for not pressed)

    /**
     * @brief Reads and returns the current state of the button.
     * 
     * This function reads the digital value of the button pin and updates the state
     * of the button accordingly.
     * 
     * @return True if the button is pressed, false otherwise.
     */
    bool getState() {
      int button_value = digitalRead(pin);
      state = (button_value == HIGH);
      return state;
    }

    /**
     * @brief Initializes the button pin.
     * 
     * This function sets the button pin mode to INPUT_PULLDOWN.
     */
    void initialize() {
      pinMode(pin, INPUT_PULLDOWN);
    }
};

#endif  // BUTTON_H
