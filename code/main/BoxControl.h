/**
 * @file BoxControl.h
 * @brief Defines the Box class for operations related to handling different sizes and colors of boxes.
 *
 * This file contains the definition of the Box class, which is used to control the operations
 * related to picking up, identifying, and placing boxes based on size and color using a robotic arm
 * in an automated system. Methods within this class manage the complete lifecycle of box handling
 * from reaching for the box to placing it at a designated location.
 *
 * Created by: Max Westerman
 */

#ifndef BOX_CONTROL_H
#define BOX_CONTROL_H

#include "Initialization.h"

/**
 * @enum BoxSize
 * @brief Enumeration to represent the size of the box.
 */
enum BoxSize {
  LARGE,
  SMALL,
};

/**
 * @class Box
 * @brief Class to handle operations and attributes for box operations.
 *
 * The Box class encapsulates methods for manipulating boxes using a robotic arm, including
 * reaching for, grabbing, and placing boxes. It also determines the size and color of the box
 * during the operation.
 */
class Box {
  public:
    BoxSize size = SMALL;         ///< Default size for testing individual methods.
    Color color = RED;            ///< Default color for testing individual methods.
    unsigned long elapsedTime;    ///< Time taken during the grab operation.

    float GRAB_SPEED = 80;        ///< Speed for grabbing the box.
    float RELEASE_SPEED = 80;     ///< Speed for releasing the box.
    float ARM_SPEED = 80;         ///< Speed for major arm motions.
    float LARGE_BOX_TIME = 700;   ///< Time threshold to differentiate between large and small boxes.

    /**
     * @brief Method to initiate reaching for the box.
     */
    void _reachFor() {
      Serial.println("= Reaching for box =");
      gripper.speed = 100;
      gripper.move(100);
      arm.speed = 50;
      arm.move(100);
    }

    /**
     * @brief Method to grab the box.
     */
    void _grab() {
      Serial.println("= Grabbing box =");
      gripper.speed = 100;
      gripper.move(100);
      gripper.speed = GRAB_SPEED;

      unsigned long startTime = millis(); // Start the timer to sense box size
      float angle = gripper.current_angle; // Check angle to increment
      while (!button.getState()) {
        angle -= 0.1; // Reduce the angle, closing the gripper
        gripper.move(angle);
        if (0 >= angle) {
          break; // Avoid servo damage
        }
      }

      unsigned long endTime = millis();
      elapsedTime = endTime - startTime;
      Serial.println(elapsedTime);
    }

    /**
     * @brief Method to determine the size of the box based on grip time.
     */
    void _getSize() {
      Serial.println("= Getting box size =");
      size = elapsedTime < LARGE_BOX_TIME ? LARGE : SMALL;
      Serial.println(size == LARGE ? "Box Size: Large Box" : "Box Size: Small Box");
    }

    /**
     * @brief Method to determine the color of the box using a sensor on the gripper.
     */
    void _getColor() {
      Serial.println("= Getting box color =");
      color = gripperColor.getColor();
      Serial.print("Box Color: ");
      Serial.println(gripperColor.color);
    }

    /**
     * @brief Method to elevate the box after grabbing.
     */
    void _pickup() {
      Serial.println("= Elevating the box =");
      arm.speed = 100;
      arm.move(80);
    }

    /**
     * @brief Public method to execute the full procedure of procuring the box.
     */
    void procure() {
      Serial.println("== Procuring Box ==");
      _reachFor();
      _grab();
      _getColor();
      _getSize();
      _pickup();
    }

    /**
     * @brief Public method to place the box at the designated location.
     */
    void place() {
      Serial.println("== Placing the box ==");
      arm.speed = ARM_SPEED;
      arm.move(95);
      gripper.speed = RELEASE_SPEED;
      gripper.move(100);
      delay(100);
      arm.speed = 100;
      arm.move(0);
      delay(100);
    }
};

Box box; ///< Global instance of Box, available for immediate use.

#endif // BOX_CONTROL_H