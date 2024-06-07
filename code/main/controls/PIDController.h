#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
// =====================

#include <Arduino.h>

class PIDController {
  private:
    float previousError;
    float integral;
    unsigned long previousTime;

  public:
    float Kp;
    float Ki;
    float Kd;

    PIDController() {
      // Default constructor with no parameters
    }

    void initialize() {
      previousError = 0;
      integral = 0;
      previousTime = millis();
    }

    float compute(float error) {
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