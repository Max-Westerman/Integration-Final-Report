#ifndef UTILS_H
#define UTILS_H
// =====================

#include <Arduino.h>

/**
 * @file Utils.h
 * @brief Utility functions for Arduino projects.
 * 
 * This file contains utility functions that can be used in various Arduino projects.
 * Currently, it includes a function to end the program by entering an infinite loop.
 * 
 * Created by: [Your Name]
 * Date: [Date]
 * 
 * Modifications:
 * Date: [Date] [Description of modification]
 */

/**
 * @brief Ends the program by printing a message and entering an infinite loop.
 * 
 * This function prints "Ended program." to the serial monitor and then enters
 * an infinite loop with a delay of 1000 milliseconds in each iteration, effectively
 * stopping the program from proceeding further.
 */
void endProgram() {
  Serial.println("Ended program.");
  while (true) {  // Enter an infinite loop
    delay(1000);  // Delay for 1000 milliseconds
  }
}

#endif  // UTILS_H
