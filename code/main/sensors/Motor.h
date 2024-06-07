#ifndef MOTOR_H
#define MOTOR_H
// 116 RPM Planetary Gear Motor | ServoCity
// =====================

#include <Arduino.h>
#include <vector>

// Type definitions for readability
typedef float percent;
typedef int ms;
typedef float cm_per_sec;

// Enum for motor movement types
enum MoveType {
  OFF,
  FORWARD,
  BACKWARD,
};

// Structure to hold calibration data for the motor
struct MotorCalibration {
  double speed_percent;
  double pwm_percent; // pwm_percent percentage (0-100)
  double speed;    // Corresponding distance or factor in cm/sec
};

// Motor class definition
class Motor {
public:
  int enPin;          // Enable pin for motor driver
  int in1Pin, in2Pin; // Input pins for motor direction control
  bool debug = false; // Debug flag for additional logging
  const char* label;  // Label for identifying the motor (optional)
  unsigned long lastUpdateTime; // Variable to store the last update time
  MoveType currentMoveType; // Variable to store the current move type
  percent currentSpeed; // Variable to store the current speed percentage
  float totalDistance; // Variable to store the total distance driven

  // CALIBRATION DATA MUST BE SORTED HIGHEST pwm_percent FIRST TO LAST.
  std::vector<MotorCalibration> calibrations; // Vector to hold calibration data

  // Initialize the motor pins and set initial state to OFF
  void initialize() {
    pinMode(enPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    drive(OFF, 0); // Ensure motor is off at initialization
    lastUpdateTime = millis(); // Initialize last update time
    currentMoveType = OFF; // Initialize current move type
    currentSpeed = 0; // Initialize current speed percentage
    totalDistance = 0; // Initialize total distance driven
  }

  // Set motor direction based on MoveType
  void motorDirection(MoveType move_type) {
    switch (move_type) {
      case OFF:
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        break;
      case FORWARD:
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        break;
      case BACKWARD:
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        break;
    }
  }

  // Set calibration data for the motor
  void setCalibrationData(const std::vector<MotorCalibration>& data) {
    calibrations = data;
  }

  // Get calibration factor based on the given speed using linear interpolation
  float getSpeed(percent pwm_percent) {
    // Return 1.0 if no calibration data is available
    if (calibrations.empty()) return 1.0;

    // Find the correct interval for interpolation
    for (size_t i = 0; i < calibrations.size() - 1; ++i) {
      if (pwm_percent >= calibrations[i].pwm_percent && pwm_percent <= calibrations[i + 1].pwm_percent) {
        // Linear interpolation formula
        double x1 = calibrations[i].pwm_percent;
        double x2 = calibrations[i + 1].pwm_percent;
        double y1 = calibrations[i].speed;
        double y2 = calibrations[i + 1].speed;
        return y1 + (y2 - y1) * (pwm_percent - x1) / (x2 - x1);
      }
    }

    // Return 1.0 if speed is out of the calibration data range
    return 1.0;
  }

  // Get calibration factor based on the given speed_percent using linear interpolation
  float getPercentPwm(percent speed_percent) {
    // Return 1.0 if no calibration data is available
    if (calibrations.empty()) return 1.0;

    // Find the correct interval for interpolation
    for (size_t i = 0; i < calibrations.size() - 1; ++i) {
      if (speed_percent >= calibrations[i].speed_percent && speed_percent <= calibrations[i + 1].speed_percent) {
        // Linear interpolation formula
        double x1 = calibrations[i].speed_percent;
        double x2 = calibrations[i + 1].speed_percent;
        double y1 = calibrations[i].pwm_percent;
        double y2 = calibrations[i + 1].pwm_percent;
        return y1 + (y2 - y1) * (speed_percent - x1) / (x2 - x1);
      }
    }

    // Return 1.0 if speed_percent is out of the calibration data range
    return 1.0;
  }

  // Update the total distance driven based on the current speed and elapsed time
  void updateDistance() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastUpdateTime;
    float speed = getSpeed(currentSpeed); // Get the current speed in cm/s
    totalDistance += (speed * elapsedTime) / 1000.0; // Update the total distance in cm
    lastUpdateTime = currentTime; // Update the last update time
  }

  // Drive the motor with a given speed and apply calibration factor
  void drive(MoveType move_type, percent speed) {
    updateDistance(); // Update the distance before changing the state
    int pwm_percent = constrain((255 * speed) / 100, 0, 255);
    motorDirection(move_type);
    analogWrite(enPin, pwm_percent);
    currentMoveType = move_type; // Store the current move type
    currentSpeed = getSpeed(pwm_percent); // Store the current speed percentage
  }

  // Drive the motor with a specific percent speed
  void driveSpeed(MoveType move_type, percent speed) {
    updateDistance(); // Update the distance before changing the state
    float calibrationFactor = getPercentPwm(speed);
    int pwm_percent = constrain((255 * calibrationFactor) / 100, 0, 255);
    motorDirection(move_type);
    analogWrite(enPin, pwm_percent);
    currentMoveType = move_type; // Store the current move type
    currentSpeed = getSpeed(pwm_percent); // Store the current speed percentage
  }

  // Get the total distance driven
  float getTotalDistance() const {
    return totalDistance;
  }

  // Test the motor by running it forward, backward, and then stopping
  void motorTest() {
    drive(FORWARD, 50); // Drive motor forward at 50% speed
    delay(500);              // Wait for 500 milliseconds
    drive(BACKWARD, 50); // Drive motor backward at 50% speed
    delay(500);              // Wait for 500 milliseconds
    drive(OFF, 0);      // Turn off the motor
  }
};

#endif // MOTOR_H