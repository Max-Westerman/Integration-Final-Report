#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
// =====================

/**
 * @file PIDController.h
 * @brief Defines the PIDController class for implementing a PID control algorithm.
 * 
 * This file contains the definition of the PIDController class, which is used to 
 * implement a PID (Proportional-Integral-Derivative) control algorithm. The class
 * provides methods to initialize the PID controller and compute the control output 
 * based on the current error value.
 * 
 * Created by: Max Westerman
 * Date: 2024 05 22
 * 
 * Modifications:
 * [2024 05 22] Creation
 * [2024 05 27] Placed within the controls subfolder
 * [2024 05 29] Added the Arduino.h header so VSCode can detect errors correctly
 * [2024 06 04] As part of a larger migration, was placed inside of the main folder
 * [2024 06 07] Removed an unneeded blank constructor, added comments
 */

#include <Arduino.h>

class PIDController {
  private:
    float previousError;          // Stores the previous PID output for integration calculation
    float integral;               // Stores the integral of the output over time
    unsigned long previousTime;   // Stores time from previous PID output for delta time

  public:
    float Kp;     // Proportional Gain
    float Ki;     // Integral Gain
    float Kd;     // Derivative Gain

    void initialize() {
      /**
       * @brief Initializes the PID controller.
       * 
       * Sets the previous error and integral values to zero and records the initial time.
       */
      previousError = 0;
      integral = 0;
      previousTime = millis();
    }

    float compute(float error) {
      /**
       * @brief Computes the control output based on the current error.
       * 
       * This method calculates the PID control output using the proportional, integral,
       * and derivative gains and the current error value.
       * 
       * @param error The current error value.
       * @return The computed control output.
       */
      unsigned long currentTime = millis();
      float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds

      integral += error * deltaTime;
      float derivative = (deltaTime > 0) ? (error - previousError) / deltaTime : 0;

      float output = Kp * error + Ki * integral + Kd * derivative;

      previousError = error;
      previousTime = currentTime;

      return output;
    }
};

#endif