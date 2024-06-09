/**
 * @file ObstacleAvoidance.h
 * @brief Header file for the ObstacleAvoidance class, managing obstacle detection and navigation.
 *
 * This file contains the class definition for ObstacleAvoidance, which uses ultrasonic sensors
 * and color detection to navigate around obstacles, maintain proper distances from walls, and
 * execute search patterns.
 */

#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

#include "Motion.h"

class ObstacleAvoidance {
public:
  /**
   * @enum ObstacleFlag
   * @brief States used to track the obstacle search and navigation process.
   */
  enum ObstacleFlag {
    SEARCH_FORWARD, ///< State to search for obstacles moving forward.
    SEARCH_LEFT,    ///< State to search for obstacles moving left.
    SEARCH_RIGHT,   ///< State to search for obstacles moving right.
    FOUND_END,      ///< State indicating that the end has been found.
  };

  cm MIN_DISTANCE = 15;                              ///< Closest desired distance to the wall.
  cm MAX_DISTANCE = 45;                              ///< Maximum desired distance from the wall.
  cm MEAN_DISTANCE = (MAX_DISTANCE + MIN_DISTANCE)/2;///< Optimal distance from the wall.
  cm MAX_PERPENDICULAR_DIFFERENCE = MEAN_DISTANCE/1.5;///< Max distance difference for perpendicular orientation.
  unsigned long REORIENT_TIME = 500;                 ///< Time interval for reorientation checks.
  ObstacleFlag obstacle_flag = SEARCH_FORWARD;       ///< Initial search state.
  Cardinal current_horizontal_search = RIGHT;        ///< Initial horizontal search direction.

  cm GAP_DISTANCE = 100;                             ///< Distance that triggers moving through an obstacle.
  cm PERPENDICULAR_DISTANCE_MARGIN_OF_ERROR = 2;     ///< Allowed error margin for perpendicular orientation.

  percent LEFT_SPEED = 100;                          ///< Speed for leftward "crab walk" motion.
  percent RIGHT_SPEED = 100;                         ///< Speed for rightward "crab walk" motion.
  percent FORWARD_SPEED = 100;                       ///< Speed for forward and backward motion.
  percent PERPENDICLUAR_TURN_SPEED = 80;             ///< Speed for turning to become perpendicular.
  int obstacles_cleared = 0;                         ///< Counter for obstacles cleared.
  int NUMBER_OF_OBSTACLES = 3;                       ///< Total number of obstacles to clear.
  cm CLEARANCE_SIZE = 8.5;                           ///< Clearance size for navigation.

  /**
   * @brief Checks if the robot is perpendicular to a wall using ultrasonic sensors.
   * @return True if perpendicular within margin of error, false otherwise.
   * 
   * Calculate the difference between the left and the right sensors. If we're not within
   * a specified margin of error, we're not perpendicular. If we are, we're perpendicular.
   */
  bool within_perpendicular_band() {
    cm ultrasonic_distance_difference;
    bool is_perpendicular;

    ultrasonic_distance_difference = abs(leftSonic.getDistance() - rightSonic.getDistance());
    is_perpendicular = ultrasonic_distance_difference <= PERPENDICULAR_DISTANCE_MARGIN_OF_ERROR;
    return is_perpendicular;
  }

  /**
   * @brief Checks if the robot is within the defined distance band on a given side.
   * @param direction The side (left or right) to check.
   * @return True if within the distance band, false otherwise.
   */
  bool within_band(Cardinal direction) {
    UltraSonic& sonic = (direction == LEFT) ? leftSonic : rightSonic;
    sonic.getDistance();
    bool withinBand = (sonic.distance >= MIN_DISTANCE && sonic.distance <= MAX_DISTANCE);
    return withinBand;
  }

  /**
   * @brief Adjusts the robot's orientation to become perpendicular to the nearest wall.
   */
  void become_perpendicular() {
    bool perpendicular = false;
    while (!perpendicular) {

      // the perpendicular function already gets the distance.
      perpendicular = within_perpendicular_band();

      if (leftSonic.distance > rightSonic.distance) {
        bot.turn(RIGHT,PERPENDICLUAR_TURN_SPEED);
      } else {
        bot.turn(LEFT,PERPENDICLUAR_TURN_SPEED);
      }
    }
    bot.stopMotion();
  }

  /**
   * @brief Searches horizontally for obstacles and navigates accordingly.
   * @param direction The direction (left or right) to search and navigate.
   * @return The next state for obstacle avoidance strategy.
   */
  ObstacleFlag searchHorizontal(Cardinal direction){
    const char* direction_string; // Used to print out what mode we're in
    direction_string = (direction == LEFT) ? "LEFT" : "RIGHT";

    current_horizontal_search = direction;
    UltraSonic& likeSonic = (direction == LEFT) ? leftSonic : rightSonic;
    ColorSensor& likeColor = (direction == LEFT) ? leftColor : rightColor;
    // ObstacleFlag like_search = (direction == LEFT) ? SEARCH_LEFT : SEARCH_RIGHT;
    float move_speed = (direction == LEFT) ? LEFT_SPEED : RIGHT_SPEED;

    UltraSonic& oppositeSonic = (direction == LEFT) ? rightSonic : leftSonic;
    // ColorSensor& oppositeColor = (direction == LEFT) ? rightColor : leftColor;
    ObstacleFlag opposite_search = (direction == LEFT) ? SEARCH_RIGHT : SEARCH_LEFT;

    Serial.print("==== Searching ");
    Serial.print(direction_string);
    Serial.println(" ====");

    unsigned long lastReorientTime = millis();

    while (true){
      // Check if 2 seconds have passed since the last reorientation
      bool reorient_timing = (millis() - lastReorientTime >= REORIENT_TIME);
      bool no_gap = (GAP_DISTANCE >= likeSonic.getDistance());
      bool ultrasonic_difference = (likeSonic.getDistance() - oppositeSonic.getDistance()) <= MAX_PERPENDICULAR_DIFFERENCE;
      if (reorient_timing && no_gap && ultrasonic_difference) {
        become_perpendicular();
        lastReorientTime = millis(); // Reset the timer
      }

      while (within_band(direction)){
        bot.move(direction,move_speed);
        if (likeColor.getColor() == WHITE) {
          leftColor.clearColorHistory();
          rightColor.clearColorHistory();
          return opposite_search;
        }
      }

      // If we're too far forward 
      if (MIN_DISTANCE >= oppositeSonic.getDistance()) {
        while (MIN_DISTANCE >= oppositeSonic.getDistance()){
          bot.move(DOWN,FORWARD_SPEED);
        }
        bool no_gap = (GAP_DISTANCE >= likeSonic.getDistance());
        bool ultrasonic_difference = (likeSonic.getDistance() - oppositeSonic.getDistance()) <= MAX_PERPENDICULAR_DIFFERENCE;
        if (no_gap && ultrasonic_difference) {
          become_perpendicular();
          lastReorientTime = millis(); // Reset the timer
        }
      }

      // If we're too far back
      if (oppositeSonic.getDistance() >= MAX_DISTANCE){
        bot.translate(direction,move_speed,CLEARANCE_SIZE);
        obstacles_cleared +=1;
        return SEARCH_FORWARD;
      }
    }
  }


  /**
   * @brief Conducts a forward search for obstacles and navigates through open spaces.
   * @return The next state for obstacle avoidance strategy.
   */
  ObstacleFlag searchForward(){
    Serial.println("==== Searching FORWARD ====");

    if (!(obstacles_cleared >= NUMBER_OF_OBSTACLES)) {

      unsigned long lastReorientTime = millis(); // Initialize the timer
      while (leftSonic.getDistance() >= MEAN_DISTANCE || rightSonic.getDistance() >= MEAN_DISTANCE ){
        bot.move(UP,FORWARD_SPEED);
        if ((millis() - lastReorientTime >= REORIENT_TIME) && (GAP_DISTANCE >= leftSonic.getDistance()) && (GAP_DISTANCE >= rightSonic.getDistance())) {
          become_perpendicular();
          lastReorientTime = millis(); // Reset the timer
        }
      }
      // Now we're close to the wall. Straighten out.
      become_perpendicular();
      if (current_horizontal_search == RIGHT) {
        return SEARCH_LEFT;
      } else {
        return SEARCH_RIGHT;
      }

    } else {
      // Increase the moving average window so we have less chance of a false positive
      leftColor.moving_average_window = 10;
      rightColor.moving_average_window = 10;
      // While the left and right sensors aren't green, keep going forwards.
      while(!((leftColor.getColor() == GREEN) && (rightColor.getColor() == GREEN))){
        bot.move(UP,FORWARD_SPEED);
      }
      return FOUND_END;
    }
  }

  /**
   * @brief Handles the detection of a "green" end signal, concluding the search.
   * @return The FOUND_END state.
   */
  ObstacleFlag foundGreen(){
    Serial.println("==== Found Green ====");

    Serial.println("END SEARCHING, GREEN FOUND");
    bot.translate(UP,FORWARD_SPEED,VERTICAL_BOT_LENGTH);
    endProgram();
    return FOUND_END;
  }

  /**
   * @brief Runs the obstacle avoidance process, transitioning between different states.
   */
  void run(){

    leftColor.moving_average_window = 8;
    rightColor.moving_average_window = 8;
    
    switch (obstacle_flag) {
      case SEARCH_RIGHT:
        obstacle_flag = searchHorizontal(RIGHT);
        break;
      case SEARCH_LEFT:
        obstacle_flag = searchHorizontal(LEFT);
        break;
      case SEARCH_FORWARD:
        obstacle_flag = searchForward();
        break;
      case FOUND_END:
        obstacle_flag = foundGreen();
    }
  }
};

// External instance of the ObstacleAvoidance class.
ObstacleAvoidance obstacleAvoidance;

#endif  // OBSTACLECONTROL_H
