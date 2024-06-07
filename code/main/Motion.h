
#ifndef MOTION_H
#define MOTION_H
// =====================

#include "Initialization.h"

typedef float cm;

enum Cardinal {
  UP,
  DOWN,
  LEFT,
  RIGHT,
};

class Movement {
public:
  /*
  There may be a difference in horizontal vs vertical percent_pwm. In our case, the
  top motors are torque and the side motors are percent_pwm. Thus, we need a normalizer.
  This is the vertical percent_pwm / horizontal percent_pwm.
  */
  float VERTICAL_VS_HORIZONTAL = 3.9;
  float VERTICAL_LENGTH = 12;
  float HORIZONTAL_LENGTH = 9;
  float horizontal_distance = 0;
  float vertical_distance = 0;

  float getHorizontalDistance() {
    topMotor.updateDistance();
    bottomMotor.updateDistance();
    horizontal_distance = (topMotor.getTotalDistance() + bottomMotor.getTotalDistance()) / 2;
    return horizontal_distance;
  }

  float getVerticallDistance() {
    leftMotor.updateDistance();
    rightMotor.updateDistance();
    vertical_distance = (leftMotor.getTotalDistance() + rightMotor.getTotalDistance()) / 2;
    return vertical_distance;
  }

  void stopMotion() {
    topMotor.drive(OFF, 0);
    bottomMotor.drive(OFF, 0);
    leftMotor.drive(OFF, 0);
    rightMotor.drive(OFF, 0);
  }

  void move(Cardinal direction,percent percent_pwm){
    switch (direction) {
      case UP:
        topMotor.drive(OFF,0);
        bottomMotor.drive(OFF,0);
        leftMotor.drive(FORWARD, percent_pwm);
        rightMotor.drive(FORWARD, percent_pwm);
        break;
      case DOWN:
        topMotor.drive(OFF,0);
        bottomMotor.drive(OFF,0);
        leftMotor.drive(BACKWARD, percent_pwm);
        rightMotor.drive(BACKWARD, percent_pwm);
        break;
      case  LEFT:
        topMotor.drive(FORWARD, percent_pwm);
        bottomMotor.drive(BACKWARD, percent_pwm);
        leftMotor.drive(OFF, 0);
        rightMotor.drive(OFF, 0);
        break;
      case RIGHT: 
        topMotor.drive(BACKWARD, percent_pwm);
        bottomMotor.drive(FORWARD, percent_pwm);
        leftMotor.drive(OFF, 0);
        rightMotor.drive(OFF, 0);
        break;
    }
  }

  void movePercent(Cardinal direction,percent percent_speed){
    // Move relative to a percent speed
    switch (direction) {
      case UP:
        topMotor.drive(OFF,0);
        bottomMotor.drive(OFF,0);
        leftMotor.driveSpeed(FORWARD, percent_speed);
        rightMotor.driveSpeed(FORWARD, percent_speed);
        break;
      case DOWN:
        topMotor.drive(OFF,0);
        bottomMotor.drive(OFF,0);
        leftMotor.driveSpeed(BACKWARD, percent_speed);
        rightMotor.driveSpeed(BACKWARD, percent_speed);
        break;
      case  LEFT:
        topMotor.driveSpeed(BACKWARD, percent_speed);
        bottomMotor.driveSpeed(BACKWARD, percent_speed);
        leftMotor.drive(OFF, 0);
        rightMotor.drive(OFF, 0);
        break;
      case RIGHT: 
        topMotor.driveSpeed(FORWARD, percent_speed);
        bottomMotor.driveSpeed(FORWARD, percent_speed);
        leftMotor.drive(OFF, 0);
        rightMotor.drive(OFF, 0);
        break;
    }
  }

  // Translate the robot at a specific percent speed for a given distance
  void translate(Cardinal direction, percent percent_pwm, cm distance) {
    // Get the calibrated speed in cm/s based on the provided PWM percent
    float speed;
    float left_speed;
    float right_speed;
    float top_speed;
    float bottom_speed;

    switch (direction) {
      case UP:
      case DOWN:
        left_speed = leftMotor.getSpeed(percent_pwm);
        right_speed = rightMotor.getSpeed(percent_pwm);
        speed = max(left_speed,right_speed)/1.1;
        break;
      case RIGHT:
      case LEFT:
        top_speed = topMotor.getSpeed(percent_pwm);
        bottom_speed = bottomMotor.getSpeed(percent_pwm);
        speed = max(top_speed,bottom_speed)/1.2;
        break;
      default:
        Serial.println("TERMINAL ERROR: An incorrect direction was set for the Motion.translate function.");
        speed = 0;
        break;
    }

    // Calculate the duration in milliseconds to cover the given distance
    ms duration = (ms)((distance / speed) * 1000);
    move(direction,percent_pwm);
    delay(duration);
    stopMotion();
  }

  void turn(Cardinal direction,percent percent_pwm){
    switch (direction) {
      case UP:
        Serial.println("Shouldn't be turning Up.");
        break;
      case DOWN:
        Serial.println("Shouldn't be turning Down.");
        break;
      case  RIGHT:
        topMotor.drive(BACKWARD, percent_pwm*VERTICAL_VS_HORIZONTAL);
        bottomMotor.drive(BACKWARD, percent_pwm*VERTICAL_VS_HORIZONTAL);
        leftMotor.drive(FORWARD, percent_pwm);
        rightMotor.drive(BACKWARD, percent_pwm);
        break;
      case LEFT: 
        topMotor.drive(FORWARD, percent_pwm*VERTICAL_VS_HORIZONTAL);
        bottomMotor.drive(FORWARD, percent_pwm*VERTICAL_VS_HORIZONTAL);
        leftMotor.drive(BACKWARD, percent_pwm);
        rightMotor.drive(FORWARD, percent_pwm);
        break;
    }
  }
};

// Initializes the bot class so other files can use it
Movement bot;
#endif

