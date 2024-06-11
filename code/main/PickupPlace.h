/**
 * @file PickupPlace.h
 * @brief Header file for the PickupPlace class, handling object manipulation following a
 * line navigation system.
 *
 * This file defines the PickupPlace class, which is responsible for picking up an object,
 * following a path to transport it, and placing the object according to its properties.
 * It uses line following, object detection, and precise movement to perform these tasks.
 */

#ifndef PICKUP_PLACE_H
#define PICKUP_PLACE_H

#include <Arduino.h>
#include "Initialization.h"   // Initialization of sensors and controls
#include "BoxControl.h"       // Controls for the object manipulation
#include "LineFollowing.h"    // Line following functionalities
#include "Motion.h"           // Basic motion control

/**
 * @class PickupPlace
 * @brief Manages the sequence of operations for picking up and placing objects, utilizing
 * sensor input and motor output.
 *
 * The PickupPlace class orchestrates a complex sequence of actions involving navigation
 * to an object, acquiring it, navigating along a designated path, and placing the object
 * based on detected characteristics. It interacts with a variety of sensors and actuators
 * to achieve precise and reliable operation.
 */
class PickupPlace {
public:
  float background_distance = 45;          ///< Max range for Ultrasonic sensors.
  float i_beam_approach_distance = 15;     ///< Distance for stopping at platform.
  float i_beam_approach_speed = 70;        ///< Speed to approach the platforms.
  float starting_line_catch_speed = 80;    ///< Initial speed to locate the starting line.
  int window_size = 4;                     ///< Moving average window size for readings.

  float following_speed = 70;              ///< Standard speed for following lines.
  float horizontal_centering_speed = 70;   ///< Speed for horizontal adjustments.

  IRLineFollower quickFollower;            ///< Faster, less precise line following.
  IRLineFollower carefulFollower;          ///< More careful, precise line following.

  /**
   * @brief Constructs and configures two types of line followers: quick and careful.
   */
  PickupPlace() {
    // Configuration of line followers with different PID settings for different behaviors
    quickFollower.kp = 100;
    quickFollower.ki = 0;
    quickFollower.kd = 29;
    quickFollower.base_speed = 100;
    quickFollower.turn_pwm = 80;
    quickFollower.turn_delay = 200;
    quickFollower.if_catch_lines = true;
    quickFollower.reverse_wheels = true;

    carefulFollower.kp = 90;
    carefulFollower.ki = 0;
    carefulFollower.kd = 5;
    carefulFollower.base_speed = 70;
    carefulFollower.turn_pwm = 30;
    carefulFollower.turn_delay = 300;
    carefulFollower.if_catch_lines = true;
    carefulFollower.reverse_wheels = true;
  }

  /**
   * @brief Moves the robot towards the platform until it reaches the specified
   * i_beam_approach_distance.
   * 
   * Uses ultrasonic sensors to measure distance to the platform and moves forward until
   * the distance is less than the threshold.
   */
  void approachPlatform(){
    Serial.println("== Approaching Platform ==");
    // While the left sonic distance is greater than the trigger, keep moving forwards.
    while (middleSonic.getDistance() > i_beam_approach_distance){
      bot.move(UP, i_beam_approach_speed);
    }
    bot.stopMotion();
  }

  /**
   * @brief Aligns the robot on the initial green line based on detected box color.
   * 
   * Moves backwards until a green line is detected and then centers on the line based on
   * the box's color.
   */
  void orientOnGreenLine(){
    Serial.println("== Orienting on the green line ==");

    leftColor.moving_average_window = 25;
    rightColor.moving_average_window = 25;
    // Keep moving down until we see the green starting
    while (!((leftColor.getColor() == GREEN) || (rightColor.getColor () == GREEN))) {
      bot.move(DOWN, starting_line_catch_speed);
    }

    // Center on the green line in the direction related to the box color.
    if (box.color == BLUE) {
      centerOnLine (LEFT, GREEN);
    }
    else if (box.color == RED) {
      centerOnLine(RIGHT, GREEN);
    }
    bot.stopMotion();
  }

  /**
   * @brief Navigates to the box colored line from the starting position.
   * 
   * Moves forward from the green line and switches to the color line specific to the box's
   * attributes.
   */
  void goToStartingLine() {
    Serial.println("== Going to Starting Line ==");
    leftColor.moving_average_window = 25;
    rightColor.moving_average_window = 25;
    leftColor.clearColorHistory(); // Clear the color history for a clean slate
    rightColor.clearColorHistory();

    // The green IR follower is buggy because the tape and tarp read similarly.
    // Thus to avoid any overcorrections, we'll turn off the aggresive wheel reversing.

    // Keep moving forwards until a color is found on the respective sensor.
    irArray.setColor(GREEN);
    if (box.color == BLUE) {
      while ((leftColor.getColor() != BLUE)){
        carefulFollower.follow(GREEN);
      }
    }
    else if (box.color == RED){
      while ((rightColor.getColor() != RED)){
        carefulFollower.follow(GREEN);
      }
    }
    bot.stopMotion();
  }

  /**
   * @brief Orients the robot parallel to the starting line for following.
   * 
   * After reaching the line, adjusts the robot to align perfectly with it for precise line following.
   */
  void orientOnStartingLine() {
    /*
    Our robot is now perpendicular to the starting line, and we want to become centered upon it.
    */
    Serial.println("== Orient on Starting Line =");
    bot.translate(DOWN, 60, 6);

    // Using the box color, center ourself on the colored line.
    if (box.color == BLUE) {
      centerOnLine(LEFT, BLUE);
      bot.turn(LEFT,30);
      delay(200);
      bot.stopMotion();
    } else if (box.color == RED) {
      centerOnLine(RIGHT, RED) ;
      bot.turn(RIGHT,30);
      delay(200);
      bot.stopMotion();
    }
    bot.stopMotion();
  }

  /**
   * @brief Follows the colored line using a line follower until a termination (green) line is
   * encountered.
   * 
   * Continuously follows the line at a set speed and checks for the green line that indicates the
   * end of the path.
   */
  void followUntilGreen(){
    Serial.println("== Following until the green line =");
    rightColor.moving_average_window = window_size; // change our moving window to smaller as we're moving faster.
    leftColor.moving_average_window = window_size;
    leftColor.clearColorHistory();
    rightColor.clearColorHistory();

    // Once the left and right and middle are reading green, stop the cart, indicating the line has been found.
    irArray.setColor(box.color);
    while (!((leftColor.getColor() == GREEN) && (rightColor.getColor() == GREEN) && (middleColor.getColor() == GREEN))){
      quickFollower.follow(box.color);
    }
    bot.stopMotion();
  }

  /**
   * @brief Manages navigation at a fork, directing the robot based on the box size.
   * 
   * Depending on the box's size, chooses a direction at a fork in the line and follows it to the
   * designated drop-off point.
   */
  void navigateFork(){
    /*
    We're now on the green trigger line after the main line following portion of the test. There's
    a fork indicating we go left or right based on the box size.
    */
    Serial.println("== Navigating the final fork ==");
    // Initialize windows to ensure modularity.
    rightColor.moving_average_window = window_size;
    leftColor.moving_average_window = window_size;
    middleColor.moving_average_window = window_size;
    // Move off of the green line by 9 cm so we're off of the green line.
    bot.translate(UP,100,9);
    // Now that we're off the line, we can move until our center color sensor sees the correct line.
    // This makes sure that we're now lined up with the line if we became off-centered from the main line sensing portion
    middleColor.clearColorHistory(); // Clear as this is a new operation.
    centerLine(box.color);

    leftColor.clearColorHistory();  // Clear as this is a new operation.
    rightColor.clearColorHistory();  // Clear as this is a new operation.
    irArray.setColor(box.color);
    carefulFollower.if_catch_lines = false; // We don't want drastic movements, we want to be centered.
    // Keep following the line until both sensors are the line color. This indicates a fork.
    while (((leftColor.getColor() != box.color) && (rightColor.getColor() != box.color))){
      carefulFollower.follow(box.color);
    }
    // We need to clear the line so we can see with our middle sensor again.
    bot.translate(UP, 100, VERTICAL_BOT_LENGTH/2);
    bot.stopMotion();
  }



  /**
   * @brief Centers the robot on the final part of the path after navigating through the fork.
   * 
   * Ensures the robot is aligned with the final segment of the line for accurate placement of the object.
   */
  void orientOnFinalLine(){
    /*
    We're now slightly above the horizontal portion of the final fork. We want to move left or right
    depending on the size of the box.
    */
    Serial.println("== Orienting on the final line =");
    middleColor.clearColorHistory();    // Clear as this is a new operation.
    middleColor.moving_average_window = window_size;
    
    for (int i = 0; i < window_size; ++i) {
        middleColor.getColor();
    }

    // While the middle sensor isn't reading the line color, move in that direction.
    if (box.size == LARGE) {
      // If the box is large, go to the lefthand line
      Serial.println("Searching left on the fork...");
      while (middleColor.getColor() != box.color){
        bot.move(LEFT, horizontal_centering_speed);
      }
      // Shimmy slightly as the middle sensor will read the color on the edge of the tape. THis centers it.
      bot.translate(LEFT, horizontal_centering_speed,2);
    }
    else if (box.size == SMALL) {
      // If the box is small, go to the righthand line
      Serial.println("Searching right on the fork...");
      while(middleColor.getColor() != box.color){
        bot.move(RIGHT, horizontal_centering_speed);
      }
      // Shimmy slightly as the middle sensor will read the color on the edge of the tape. THis centers it.
      bot.translate(RIGHT, horizontal_centering_speed, 2);
    }

    middleColor.moving_average_window = window_size;
    bot.stopMotion();
  }

  /**
   * @brief Approaches the final platform, stopping within a precise distance for object placement.
   * 
   * Uses careful navigation to approach the platform and prepares for object placement by aligning
   * with the platform edge.
   */
  void approachEndPlatform(){
    Serial.println("== Approaching the last platform ==");
    /*
    The platform is now in front of us, but the left ultrasonic sensor is not in the middle of the bot.
    Thus, to sense when we've reached the platform, we should use the ir sensor. This function will
    stop once we've found it.
    */

    // While the platform ir sensor isn't reading, follow the line and move forward.
    while (middleSonic.getDistance() >= i_beam_approach_distance) {
      carefulFollower.follow(box.color);
    }
    bot.stopMotion();
  }

  /**
   * @brief Returns to the main green line after placing the object.
   * 
   * Navigates back to the green line, preparing for potential additional tasks or to conclude the
   * operation.
   */
  void returnToGreen(){
    middleColor.moving_average_window = 25;

    middleColor.clearColorHistory();
    while (middleColor.getColor() != BLACK){
      bot.turn(RIGHT,60);
    }

    middleColor.clearColorHistory();
    while (middleColor.getColor() != box.color){
      bot.turn(RIGHT,60);
    }
    middleColor.moving_average_window = window_size;
    bot.stopMotion();
  }

  /**
   * @brief Returns to the fork in the line to continue navigation or repeat the process.
   * 
   * Re-navigates to a critical decision point in the path to either continue further tasks or to
   * reattempt alignment.
   */
  void returnToFork(){
    Serial.println("== Going back to the fork ==");
    /*
    The platform is now in front of us, but the left ultrasonic sensor is not in the middle of the bot.
    Thus, to sense when we've reached the platform, we should use the ir sensor. This function will
    stop once we've found it.
    */

    // While the platform ir sensor isn't reading, follow the line and move forward.
    leftColor.clearColorHistory();
    rightColor.clearColorHistory();
    while ((leftColor.getColor() != box.color) && (rightColor.getColor() != box.color)) {
      carefulFollower.follow(box.color);
    }
    bot.stopMotion();
    bot.translate(UP, 100, VERTICAL_BOT_LENGTH/2);

    middleColor.moving_average_window = 25;
    leftColor.moving_average_window = 25;
    rightColor.moving_average_window = 25;

    middleColor.clearColorHistory();
    leftColor.clearColorHistory();
    rightColor.clearColorHistory();
    if (box.size == LARGE){
      while (leftColor.getColor() != box.color){
        bot.move(LEFT,horizontal_centering_speed);
      }
      bot.stopMotion();
      while (middleColor.getColor() != box.color){
        bot.move(RIGHT,horizontal_centering_speed);
      }
      bot.stopMotion();
      while (middleColor.getColor() != box.color){
        bot.move(RIGHT,horizontal_centering_speed);
      }
    }
    bot.stopMotion();
  }

  /**
   * @brief Executes the full sequence of picking up and placing an object.
   * 
   * Orchestrates the entire process from initial approach to final placement, including all necessary adjustments and alignments.
   */
  void run(){
    Serial.println("| ==== Running Pickup & Place ==== |"); 
    // Runs all of the functions in order
    approachPlatform();         // Moves on the y axis until it gets close to the platform.
    box.procure();              // Grab, get properties, and raise the box.

    orientOnGreenLine();        // Drive back until the starting green line, then rotate to the colored line
    goToStartingLine();         // Drive until the colored line to follow is found
    orientOnStartingLine();     // Orient so the bot is parallel with the follow line
    followUntilGreen();         // Follow the colored line until the green line designates termination

    navigateFork();             // Go to the horizontal piece of the fork    
    orientOnFinalLine();        // Translate horizontally to get the robot on the fork spline

    approachEndPlatform();      // Moves on the y axis until it gets close to the platform.
    box.place();                // Place the box

    returnToGreen();
    returnToFork();
    followUntilGreen();
    // Move off of the green line by 5 cm so we're off of the green line.
    bot.translate(UP,100,5);
    followUntilGreen();
    
    bot.stopMotion();
    endProgram();
  }

};

// External instantiation of the PickupPlace class for use in other files.
PickupPlace pickupPlace;

#endif // PICKUP_PLACE_H