#ifndef MOTION_H
#define MOTION_H

/**
 * @file Motion.h
 * @brief Header file for the Movement class, which controls robot movement mechanics.
 *
 * This file contains the class definition for managing the movement of the robot,
 * including directional movement, stopping, turning, and translating over specified
 * distances. It provides interfaces to higher-level motion control operations based on
 * specified parameters.
 */

#include "Initialization.h"

typedef float cm; ///< Define a custom type for measurements in centimeters.

/**
 * @enum Cardinal
 * @brief Enumeration for directionality in robot movements.
 */
enum Cardinal {
  UP,    ///< Represents upward movement or forward direction.
  DOWN,  ///< Represents downward movement or backward direction.
  LEFT,  ///< Represents leftward movement.
  RIGHT  ///< Represents rightward movement.
};

/**
 * @class Movement
 * @brief Manages directional control and motion of the robot.
 *
 * The Movement class encapsulates methods for moving the robot in specific directions,
 * stopping the robot, and performing complex maneuvers such as turning and translating
 * at various speeds and durations.
 */
class Movement {
public:
  float VERTICAL_VS_HORIZONTAL = 3.9; ///< Normalizer ratio for vert and horiz motor PWM.
  float VERTICAL_LENGTH = 12;         ///< Length of the robot's vertical axis in cm.
  float HORIZONTAL_LENGTH = 9;        ///< Length of the robot's horizontal axis in cm.
  float horizontal_distance = 0;      ///< Accumulated horizontal distance traveled.
  float vertical_distance = 0;        ///< Accumulated vertical distance traveled.

  /**
   * @brief Calculate the total horizontal distance traveled by the robot.
   * @return The calculated horizontal distance.
   */
  float getHorizontalDistance() {
    topMotor.updateDistance();
    bottomMotor.updateDistance();
    horizontal_distance = (
      topMotor.getTotalDistance() + bottomMotor.getTotalDistance()) / 2;
    return horizontal_distance;
  }

  /**
   * @brief Calculate the total vertical distance traveled by the robot.
   * @return The calculated vertical distance.
   */
  float getVerticallDistance() {
    leftMotor.updateDistance();
    rightMotor.updateDistance();
    vertical_distance = (leftMotor.getTotalDistance() + rightMotor.getTotalDistance()) / 2;
    return vertical_distance;
  }

  /**
   * @brief Stops all motor activity, effectively halting the robot.
   */
  void stopMotion() {
    topMotor.drive(OFF, 0);
    bottomMotor.drive(OFF, 0);
    leftMotor.drive(OFF, 0);
    rightMotor.drive(OFF, 0);
  }

  /**
   * @brief Moves the robot in a specified cardinal direction at a given PWM percentage.
   * @param direction The cardinal direction to move.
   * @param percent_pwm The PWM percentage to apply to the motors.
   */
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

  /**
   * @brief Moves the robot in a givencardinal direction based on a % of maximum speed.
   * @param direction The cardinal direction to move.
   * @param percent_speed The speed percentage to apply.
   */
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

  /**
   * @brief Translates the robot in a specified direction for a given distance at a
   * specified speed.
   * @param direction The cardinal direction to move.
   * @param percent_pwm The PWM percentage to apply to the motors.
   * @param distance The distance to move in centimeters.
   */
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
        Serial.println("TERMINAL ERROR");
        Serial.print("An incorrect direction was set for the");
        Serial.print("Motion.translate function.");
        speed = 0;
        break;
    }

    // Calculate the duration in milliseconds to cover the given distance
    ms duration = (ms)((distance / speed) * 1000);
    move(direction,percent_pwm);
    delay(duration);
    stopMotion();
  }

  /**
   * @brief Performs a turning maneuver in the specified direction at a given PWM
   * percentage.
   * @param direction The cardinal direction to turn towards.
   * @param percent_pwm The PWM percentage to apply to the motors.
   */
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

// Initializes the Movement class instance to be available globally.
Movement bot;

#endif // MOTION_H

