/**
 * @file MWServo.h
 * @brief Defines the MWServo class for controlling servo motors.
 *
 * This file contains the definition of the MWServo class, designed to handle and control
 * the motion of servo motors in robotic applications, specifically targeting models from ServoCity's
 * 2000 series dual mode servo (25-2, Torque) and (25-3, Speed).
 *
 * Created by: Max Westerman
 */

#ifndef MWSERVO_H
#define MWSERVO_H

#include <Servo.h>
#include <Arduino.h>

typedef float degree; ///< Represents angles in degrees.

/**
 * @class MWServo
 * @brief Class to control a servo's movement and behavior.
 *
 * The MWServo class provides methods for setting up and controlling the motion
 * of a servo motor. It allows for specifying the angle, speed, and the smoothness
 * of movement.
 */
class MWServo {
  public:
    int pin; ///< The Arduino pin number to which the servo is connected.
    float speed = 80; ///< Speed of the servo as a percentage (0-100).
    float current_angle; ///< Current angle of the servo in degrees.
    float base_angle; ///< Base angle for the servo's zero position.
    float max_angle; ///< Maximum angle the servo can rotate to.
    const degree angle_step = 0.1; ///< Resolution of angle change per step, affects smoothness.
    const int maxDelayPerAngle = 100; ///< Max delay per angle at lowest speed (ms).
    const int minDelayPerAngle = 5; ///< Min delay per angle at highest speed (ms).
    Servo servo; ///< Servo object to interface with hardware.

    /**
     * Writes a given angle to the servo.
     * @param angle The angle in degrees to set the servo.
     */
    void write(float angle) {
        current_angle = angle; // Update current angle
        servo.write(angle); // Command the servo to move to the specified angle
    }

    /**
     * Moves the servo to a target angle.
     * @param targetAngle Target angle as a percentage (0-100) to be mapped to actual angle.
     */
    void move(int targetAngle) {
        targetAngle = map(targetAngle, 0, 100, base_angle, max_angle); // Map percentage to angle range

        float percent_speed = speed / 100.0; // Convert speed from percentage to fraction

        unsigned long baseDelay = (unsigned long)((1 - percent_speed) * (maxDelayPerAngle - minDelayPerAngle) + minDelayPerAngle); // Calculate base delay per angle based on speed
        unsigned long stepDelay = (unsigned long)(baseDelay * angle_step); // Adjust step delay based on angle step

        if (current_angle < targetAngle) {
            for (degree angle = current_angle + angle_step; angle <= targetAngle; angle += angle_step) {
                write(angle); // Write new angle to servo
                delay(stepDelay); // Delay to pace the movement
            }
        } else {
            for (degree angle = current_angle - angle_step; angle >= targetAngle; angle -= angle_step) {
                write(angle); // Write new angle to servo
                delay(stepDelay); // Delay to pace the movement
            }
        }
    }

    /**
     * Initializes the servo, attaching it to a pin and setting initial conditions.
     */
    void initialize() {
        servo.attach(pin); // Attach the servo object to the specified pin
        servo.write(0); // Set initial servo position to 0 degrees
    }
};

#endif // MWSERVO_H