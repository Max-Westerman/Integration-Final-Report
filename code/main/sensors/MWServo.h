#ifndef MWSERVO_H
#define MWSERVO_H
// 2000 Series Dual Mode Servo (25-2, Torque) | ServoCity
// 2000 Series Dual Mode Servo (25-3, Speed ) | ServoCity
// =====================

#include <Servo.h> 
#include <Arduino.h>

typedef float degree;

class MWServo {
  public:
  int pin;
  float speed = 80;
  float current_angle;
  float base_angle;
  float max_angle;
  const degree angle_step = 0.1;     // How smooth the motors moves.
  const int maxDelayPerAngle = 100; // Delay per angle at speed ~0 (ms)
  const int minDelayPerAngle = 5;  // Delay per angle at speed 100 (ms)
  Servo servo;

  void write(float angle){
    // Each servo has a different "base state". For example, the arm motor's normal 0 angle is actually at 7
    // Thus, the base_angle will be subtracted.
    current_angle = angle;
    servo.write(angle);
  }

  void move(int targetAngle) {
    targetAngle = map(targetAngle,0,100,base_angle,max_angle);

    float percent_speed = speed / 100.0; // Convert from 100-0 to 1-0

    // Calculate the base delay per angle based on the speed
    unsigned long baseDelay = (unsigned long)((1 - percent_speed) * (maxDelayPerAngle - minDelayPerAngle) + minDelayPerAngle);

    // Adjust the step delay based on the angle step
    unsigned long stepDelay = (unsigned long)(baseDelay * angle_step);

    if (current_angle < targetAngle) {
      // Moving to a greater angle
      for (degree angle = current_angle + angle_step; angle <= targetAngle; angle += angle_step) {
        write(angle);
        delay(stepDelay);
      }
    } else {
      // Moving to a lesser angle
      for (degree angle = current_angle - angle_step; angle >= targetAngle; angle -= angle_step) {
        write(angle);
        delay(stepDelay);
      }
    }
  }

  void initialize(){
    servo.attach(pin);
    servo.write(0);
  }
};

#endif