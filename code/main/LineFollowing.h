/**
 * @file LineFollowing.h
 * @brief This file contains the IRLineFollower class for line following using IR sensors.
 *
 * This class manages the behavior of a robot equipped with infrared (IR) sensors for
 * following a line on the ground. It includes methods for initializing the robot, processing
 * sensor data, and controlling the motors based on computed errors using a PID controller.
 */

#ifndef LINEFOLLOWING_H
#define LINEFOLLOWING_H

#include "Initialization.h"
#include "Motion.h"
#include <functional>
#include <thread>

class IRLineFollower {
public:
  const float MAX_SPEED = 100; ///< Maximum speed of the robot.
  const float max_threshold = 1; ///< Upper threshold for error handling.
  const float min_threshold = 0.55; ///< Lower threshold for error handling.

  bool if_catch_lines = true; ///< Determines if the robot uses 'delay turns' on colors.
  bool reverse_wheels = true; ///< Flag to enable wheel reversal for turning maneuvers.
  Color follow_color; ///< Color of the line being followed.
  float error; ///< Current error value calculated by sensors.
  float pid_output; ///< Output value from the PID controller.
  percent base_speed; ///< Base speed for the robot's movement.
  percent turn_pwm; ///< PWM value for turn operations.
  float turn_delay; ///< Delay in seconds to apply after a turn.
  float kp, ki, kd; ///< PID coefficients for proportional, integral, and derivative terms.

  percent left_speed, right_speed;
  percent top_speed, bottom_speed; ///< Speed settings for each motor.
  percent left_pwm, right_pwm;
  percent top_pwm, bottom_pwm; ///< PWM settings for each motor.
  MoveType left_direction, right_direction;
  MoveType top_direction, bottom_direction; ///< Direction control for each motor.

  /**
   * @brief Follows a colored line specified by the parameter.
   * @param followed_color Color of the line to follow.
   */
  void follow(Color followed_color) {
    follow_color = followed_color;

    if (middleColor.getColor() == YELLOW) {
      irArray.setColor(YELLOW);
    } else {
      irArray.setColor(follow_color);
    }

    // Handle line catching and turning
    handleLineCatching();

    // Calculate the error and PID output
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    error = irArray.getError();  // Save the error in the class variable
    pid_output = pid.compute(error);  // Save the pid_output in the class variable

    adjustMotorSpeeds();

    // Drive the motors
    drives();
  }

  private:

  /**
   * @brief Handles the detection and response to line catching.
   */
  void handleLineCatching() {
    if (if_catch_lines) {
    if ((leftColor.getColor() == follow_color) || (rightColor.getColor() == follow_color))
    {
      if (leftColor.color == follow_color) {
        bot.turn(LEFT, turn_pwm);
        delay(turn_delay);
      } else if (rightColor.color == follow_color) {
        bot.turn(RIGHT, turn_pwm);
        delay(turn_delay);
      }
      bot.stopMotion();
      leftColor.clearColorHistory();
      rightColor.clearColorHistory();
    }
  }
  }

  /**
   * @brief Adjusts motor speeds based on the current PID error.
   */
  void adjustMotorSpeeds() {
    left_direction = FORWARD;
    right_direction = FORWARD;
    right_speed = base_speed - pid_output;
    left_speed = base_speed + pid_output;

    // Adjust the speed and direction based on the error and thresholds
    if (reverse_wheels) {
      if (error > min_threshold) {
        right_speed = base_speed * (1.0 - (error - min_threshold) /
          (max_threshold - min_threshold));
        if (error >= min_threshold + (max_threshold - min_threshold) / 2) {
          right_direction = BACKWARD;
          right_speed = base_speed * (
            1.0 - (error - min_threshold) / (max_threshold - min_threshold)
            ) +20;
        }
      }

      if (error < -min_threshold) {
        left_speed = base_speed * (1.0 + (error + min_threshold) /
          (max_threshold - min_threshold));
        if (error <= -(min_threshold + (max_threshold - min_threshold) / 2)) {
          left_direction = BACKWARD;
          left_speed = base_speed * (1.0 + (error + min_threshold) /
            (max_threshold - min_threshold)) + 20;
        }
      }
    }

    // Clamp speeds to the maximum speed limit
    right_speed = constrain(right_speed, 0, MAX_SPEED);
    left_speed = constrain(left_speed, 0, MAX_SPEED);
    right_pwm = rightMotor.getPercentPwm(right_speed);
    left_pwm = leftMotor.getPercentPwm(left_speed);

    float front_back_ratio = TOP_MOTOR_TO_IR_ARRAY_LENGTH/BOTTOM_MOTOR_TO_IR_ARRAY_LENGTH;

    if (error > 0) {
      top_pwm = topMotor.getPercentPwm(right_speed*0.7);
      bottom_pwm = bottomMotor.getPercentPwm(left_speed*front_back_ratio);
      top_direction = BACKWARD;
      bottom_direction = BACKWARD;
    } else {
      top_pwm = topMotor.getPercentPwm(left_speed*0.7);
      bottom_pwm = bottomMotor.getPercentPwm(right_speed*front_back_ratio);
      top_direction = FORWARD;
      bottom_direction = FORWARD;  
    }
  }

  /**
   * @brief Drives the motors based on the computed speeds and directions.
   */
  void drives() {  
    // Serial.println(error);
    leftMotor.drive(left_direction, left_pwm);
    rightMotor.drive(right_direction, right_pwm);

    // Top and bottom motor logic
    topMotor.drive(top_direction, top_pwm);
    bottomMotor.drive(bottom_direction, bottom_pwm);
  }
};

// Initialize the IR line follower class
IRLineFollower irFollower;

/**
 * @brief Centers the robot on line by rotating based on the used direction and line color.
 * 
 * This function stops the robot, adjusts its position to center it on the line, and then
 * continuously adjusts its rotation to maintain alignment with the line. It uses color
 * sensors to detect the line and control the robot's motion.
 * 
 * @param rotation The direction to rotate to center on the line (LEFT or RIGHT).
 * @param follow_color The color of the line to follow.
 */
void centerOnLine(Cardinal rotation, Color follow_color) {
  bot.stopMotion();
  float turn_speed = 60;
  // float forward_speed = 50;
  
  ColorSensor* colorSensor;
  if (rotation == LEFT) {
    colorSensor = &leftColor;
  } else {
    colorSensor = &rightColor;
  }

  // Stop and wait for the shock absorbers to go back to neutral position
  bot.stopMotion();
  delay(100);  

  // To normalize location on the line, keep moving up until the line can't be seen anymore
  bot.move(UP,30);
  colorSensor->clearColorHistory();
  while ((colorSensor->getColor() == follow_color)) {
  }
  bot.stopMotion();

  // Move half length forward to be in the middle of the line
  bot.translate(UP,60,8);

  // Keep rotating after the middle sees to align yourself
  if (rotation == LEFT){
    bot.turn(LEFT,turn_speed);
  } else if (rotation == RIGHT) {
    bot.turn(RIGHT,turn_speed);
  }
  middleColor.clearColorHistory();
  while (middleColor.getColor() != follow_color) {
  }
  delay(100); // Keep rotating to get truly centered.
  bot.stopMotion();  // Stop the robot's motion after detecting the color
  
  irArray.setColor(follow_color);
  while (abs(irArray.getError()) > 0.3) {
    if (rotation == LEFT){
      bot.turn(LEFT,turn_speed);
    } else if (rotation == RIGHT) {
      bot.turn(RIGHT,turn_speed);
    }
  }
}

/**
 * @brief Searches for and aligns the robot along X-axis by detecting specified line color.
 * 
 * This function attempts to locate the line by moving in a zigzag pattern. Once the line
 * is found, it stops the robot and translates it to adjust the position on the X-axis,
 * ensuring accurate alignment.
 * 
 * @param find_color The color of the line to locate and align with.
 */
void centerLine(Color find_color){
  Serial.println("== Locating Line on X axis ==");
    bool located = false;
    const float speed = 70;
    const int leftDuration = 3000;
    const int rightDuration = 1500;
    unsigned long startTime;

    for (int i = 0; i < middleColor.moving_average_window; ++i) {
      middleColor.getColor();
    }

    while (!located) {
      // Move right for a specified duration or until the platform is detected
      startTime = millis();
      while (millis() - startTime < rightDuration) {
        if (middleColor.getColor() == find_color) {
          located = true;
          // bot.translate(RIGHT,speed,2);
          bot.stopMotion();
          break;
        }
        bot.move(RIGHT,speed);
      }
      if (located) break;

      // Move right for a longer duration or until the platform is detected
      startTime = millis();
      while (millis() - startTime < leftDuration) {
        if (middleColor.getColor() == find_color) {
          located = true;
          // bot.translate(LEFT,speed,2);
          bot.stopMotion();
          break;
        }
        bot.move(LEFT,speed);
      }
      if (located) break;
      break;
  }
  bot.stopMotion();
}

#endif // LINEFOLLOWING_H
