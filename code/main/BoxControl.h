#ifndef BOX_CONTROL_H
#define BOX_CONTROL_H
// =====================

#include "Initialization.h"

// enum for box sizes
enum BoxSize {
  LARGE,
  SMALL,
};

// Class for box operations & attributes
class Box {
  public:  
  BoxSize size = SMALL;         // Box size. Initialized for testing individual methods.
  Color color = RED;           // Box color.  Initialized for testing individual methods.
  unsigned long elapsedTime;    // Time taken during the grab operation
  
  float GRAB_SPEED = 80;        // Speed for grabbing the box
  float RELEASE_SPEED = 80;
  float ARM_SPEED = 80;         // How fast the arm moves up and down for major motions 
  float LARGE_BOX_TIME = 700;   // Time threshold to differentiate between large and small boxes

  // Method to initiate reaching for the box
  void _reachFor(){
    Serial.println("= Reaching for box =");
    // Open the gripper fully at full speed before putting the arm down.
    gripper.speed = 100;
    gripper.move(100);
    // Slow down the arm and move the arm to horizontal.
    arm.speed = 50;
    arm.move(100);
  }

  void _grab(){
    Serial.println("= Grabbing box =");
    // Verifying the gripper is fully open.
    gripper.speed= 100;
    gripper.move(100);
    gripper.speed = 80;

    unsigned long startTime = millis();     // Start the timer to sense box size
    float angle = gripper.current_angle;    // Check angle so we can increment
    gripper.speed = GRAB_SPEED;

    // Slowly move the gripper closer together until the button is pressed
    while (!button.getState()){
      angle -= 0.1;         // Reduce the angle, closing the griper
      gripper.move(angle);  // Move to the new angle 
      if (0 >= angle) {
        break; // Makes sure the servo doesn't strip. Exits loop at min angle.
      }
    };

    // Compute the time taken to close the gripper
    unsigned long endTime = millis();
    elapsedTime = endTime-startTime;
    Serial.println(elapsedTime);
  }

  void _getSize(){
    // Using the time to grab the box, get the size of the box.
    Serial.println("= Getting box size =");
    if (elapsedTime < LARGE_BOX_TIME) {
      Serial.println("Box Size: Large Box");
      size = LARGE; // Box.size
    } else {
      Serial.println("Box Size: Small Box");
      size = SMALL; // Box.size
    }
  }

  void _getColor(){
    // Using the gripper color sensor, retrieve the color of the box.
    Serial.println("= Getting box color =");
    color = gripperColor.getColor(); // Box.color
    Serial.print("Box Color: ");
    Serial.println(gripperColor.color);
  }

  void _pickup(){
    Serial.println("= Elevating the box =");
    arm.speed = 100;
    arm.move(80);
  }

  void procure(){
    // Run through the sub methods on getting the box.
    Serial.println("== Procuring Box ==");
    _reachFor();
    _grab();
    _getColor();
    _getSize();
    _pickup();
  }

  void place(){
    Serial.println("== Placing the box ==");
    // Set the arm back to horizontal
    arm.speed = ARM_SPEED;
    arm.move(95);

    // Release the box
    gripper.speed = RELEASE_SPEED;
    gripper.move(100);
    delay(100);   // Wait slightly so box doesn't stick to gripper

    // Bring the arm back up, quickly, to vertical. Congrats, task completed!
    arm.speed = 100;
    arm.move(0);
    delay(100);   // Wait slightly so box doesn't stick to gripper
  }
};

Box box;

#endif