#include "Arduino.h"

#include "Initialization.h"
#include "Motion.h"
#include "ObstacleAvoidance.h"
#include "PickupPlace.h"
#include "LineFollowing.h"

void setup() {
  Serial.begin(9600);   // Start with a baud rate of 9600
  // while (!Serial); // Uncomment if debugging. Waits until serial connection is verified.
  Serial.println("| ===== Program Start ===== |");

  // Initialize Sensors
  initMotors();
  initButtons();
  initUltrasonicSensors();
  initServos();
  initIRArray();
  initPID();
  initColorSensors();
  initColorCalibrations();
  Serial.println("| ==== Setup  Complete ==== |"); 
}

void loop() {
  // obstacleAvoidance.run();
  // pickupPlace.run();
  bot.movePercent(UP,80);
  delay(2000);
  Serial.println(leftMotor.getTotalDistance());
  Serial.println(rightMotor.getTotalDistance());
  bot.stopMotion();
  endProgram();
}
