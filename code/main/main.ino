#include <Arduino.h>
#include "Initialization.h"
#include "Motion.h"
#include "ObstacleAvoidance.h"
#include "PickupPlace.h"
#include "LineFollowing.h"

/**
 * @file main.cpp
 * @brief Main entry file for the robot's operation system.
 *
 * This file initializes the hardware components and sensors of the robot and enters
 * the main control loop where specific functionalities like obstacle avoidance and
 * object manipulation are executed.
 */

/**
 * @brief Initializes the robot's modules and sensors.
 * 
 * Setup function that initializes communication settings, sensor modules,
 * motor controllers, and other necessary components to start the robot's operation. This
 * includes setting up serial communication, initializing various sensors and motors, and
 * preparing the robot for its main operational tasks.
 */
void setup() {
  Serial.begin(9600);   // Start serial communication with a baud rate of 9600
  // while (!Serial); // Ensures serial communication. If running on battery, comment out.

  Serial.println("| ===== Program Start ===== |");

  // Calls to initialization functions from included headers
  initMotors(); 
  initButtons();
  initUltrasonicSensors();
  initServos();
  initIRArray();
  initPID();
  initColorSensors();
  initColorCalibrations();

  Serial.println("| ==== Setup Complete ==== |"); 
}

/**
 * @brief Main execution loop of the robot.
 * 
 * Continuously checks and runs major functionalities of the robot such as obstacle
 * avoidance and pickup/place tasks. This loop allows the robot to perform its intended
 * tasks repeatedly as long as the robot remains powered.
 */
void loop() {
  // obstacleAvoidance.run();  ///< Uncomment to activate obstacle avoidance
  pickupPlace.run();        ///< Run pickup and placement tasks
}
