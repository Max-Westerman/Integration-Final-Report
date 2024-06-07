#ifndef INITIALIZATION_H
#define INITIALIZATION_H
// =====================

#include "Arduino.h"

#include "sensors/ColorSensor.h"
#include "sensors/Motor.h"
#include "sensors/UltraSonic.h"
#include "sensors/MWServo.h"
#include "sensors/Button.h"
#include "sensors/IRSensorArray.h"

#include "controls/PIDController.h"
#include "controls/Utils.h"

#define VERTICAL_BOT_LENGTH 31.115
#define HORIZONTAL_BOT_LENGTH 23.1775
#define TOP_MOTOR_TO_IR_ARRAY_LENGTH 5.08
#define BOTTOM_MOTOR_TO_IR_ARRAY_LENGTH 25.4

ColorSensor leftColor;
ColorSensor rightColor;
ColorSensor gripperColor;
ColorSensor middleColor;

Motor topMotor;
Motor bottomMotor;
Motor leftMotor;
Motor rightMotor;

UltraSonic rightSonic;
UltraSonic leftSonic;
UltraSonic middleSonic;

MWServo arm;
MWServo gripper;
Button button;

PIDController pid;
IRSensorArray irArray;

void initIRArray() {

  int numSensors = 7;
  irArray.numSensors = numSensors;
  irArray.sensorPins = new int[numSensors]{A3, A4, A5, A6, A7, A8, A9};

  int* onValuesBlue = new int[numSensors]{942, 962, 961, 961, 958, 972, 981};
  int* onValuesGreen = new int[numSensors]{959, 970, 969, 972, 969, 978, 983};
  int* onValuesRed = new int[numSensors]{958, 974, 974, 976, 973, 982, 988};
  int* onValuesYellow = new int[numSensors]{928, 949, 948, 952, 947, 964, 977};
  int* offValues = new int[numSensors]{1005, 1006, 1007, 1006, 1007, 1008, 1007};
  
  irArray.setCalibrationValues(RED, onValuesRed, offValues);
  irArray.setCalibrationValues(GREEN, onValuesGreen, offValues);
  irArray.setCalibrationValues(BLUE, onValuesBlue, offValues);
  irArray.setCalibrationValues(YELLOW, onValuesYellow, offValues);
  irArray.initialize();
}

void initMotors(){

  // ==== TOP ====
  // Define calibration data
  std::vector<MotorCalibration> topCalibrationData = {
    {0, 25, 5.715},
    {6.67, 30, 8.5725},
    {20, 40, 14.3933333333333},
    {33.33, 50, 19.8966666666667},
    {46.67, 60, 24.6591666666667},
    {60, 70, 28.6279166666667},
    {73.33, 80, 32.86125},
    {86.67, 90, 37.2533333333333},
  };
  topMotor.enPin = 3;
  topMotor.in1Pin = 4;
  topMotor.in2Pin = 5;
  topMotor.label = "Top motor";
  topMotor.setCalibrationData(topCalibrationData);
  topMotor.initialize();

  // ==== BOTTOM ====
  std::vector<MotorCalibration> bottomCalibrationData = {
    {0, 25, 5.57106666666667},
    {6.67, 30, 8.18303333333333},
    {20, 40, 13.7244666666667},
    {33.33, 50, 19.2616666666667},
    {46.67, 60, 22.2779166666667},
    {60, 70, 24.8179166666667},
    {73.33, 80, 26.8816666666667},
    {86.67, 90, 30.6916666666667},
  };
  bottomMotor.enPin = 9;
  bottomMotor.in1Pin = 11;
  bottomMotor.in2Pin = 10;
  bottomMotor.label = "Bottom motor";
  bottomMotor.setCalibrationData(bottomCalibrationData);
  bottomMotor.initialize();

  // ==== LEFT ====

  std::vector<MotorCalibration> leftCalibrationData = {
    {0, 19, 0},
    {1.23, 20, 2.4},
    {7.41, 25, 9.398},
    {13.58, 30, 14.4},
    {25.93, 40, 20.16},
    {38.27, 50, 27.4},
    {50.62, 60, 33.6},
    {62.96, 70, 36.18},
    {75.31, 80, 41.6},
    {87.65, 90, 44.3},
    {100, 100, 54.4},
  };
  leftMotor.enPin = 0;
  leftMotor.in1Pin = 1;
  leftMotor.in2Pin = 2;
  leftMotor.label = "Left motor";
  leftMotor.setCalibrationData(leftCalibrationData);
  leftMotor.initialize();

  // ==== RIGHT ====
  std::vector<MotorCalibration> rightCalibrationData = {
    {0, 19, 0},
    {1.23, 20, 2.4},
    {7.41, 25, 9.398},
    {13.58, 30, 13.6},
    {25.93, 40, 18.36},
    {38.27, 50, 24.6},
    {50.62, 60, 31.32},
    {62.96, 70, 32.18},
    {75.31, 80, 38.2},
    {87.65, 90, 40.5},
    {100, 100, 48.48},
  };
  rightMotor.enPin = 6;
  rightMotor.in1Pin = 7;
  rightMotor.in2Pin = 8;
  rightMotor.label = "Right motor";
  rightMotor.setCalibrationData(rightCalibrationData);
  rightMotor.initialize();

}

void initButtons(){
  button.pin = 36;
  button.initialize();
}

void initUltrasonicSensors(){

 // ==== LEFT ====

  leftSonic.trigPin = 38;
  leftSonic.echoPin = 37;
  leftSonic.soundDelay = 10;
  leftSonic.label = "Left UltraSonic Sensor";
  leftSonic.initialize();

 // ==== RIGHT ====

  rightSonic.trigPin = 27;
  rightSonic.echoPin = 28;
  rightSonic.soundDelay = 10;
  rightSonic.label = "Right UltraSonic Sensor";
  rightSonic.initialize();

 // ==== MIDDLE ====

  middleSonic.trigPin = 31;
  middleSonic.echoPin = 32;
  middleSonic.soundDelay = 10;
  middleSonic.label = "Middle UltraSonic Sensor";
  middleSonic.initialize();
}

void initServos(){

 // ==== GRIPPER ====

  gripper.pin = 29;
  gripper.base_angle = 0;
  gripper.max_angle = 60;
  gripper.initialize();
  gripper.speed = 100;
  gripper.move(100);
  
 // ==== ARM ====

  arm.pin = 30;
  arm.base_angle = 7;
  arm.max_angle = 54;
  arm.initialize();
  arm.speed = 80;
  arm.move(80);

}

void initPID(){
  pid.initialize();
}

void initColorSensors(){
  rightColor.output_frequency_0_pin = 14;
  rightColor.output_frequency_1_pin = 13;
  rightColor.color_selector_2_pin = 24;
  rightColor.color_selector_3_pin = 25;
  rightColor.out_pin = 26;

  rightColor.label = "Right";
  rightColor.frequency = 20;
  rightColor.initialize();

 // ======

  leftColor.output_frequency_0_pin = 14;
  leftColor.output_frequency_1_pin = 13;
  leftColor.color_selector_2_pin = 41;
  leftColor.color_selector_3_pin = 40;
  leftColor.out_pin = 39;

  leftColor.label = "Left";
  leftColor.frequency = 20;
  leftColor.initialize();

 // ======

  middleColor.output_frequency_0_pin = 14;
  middleColor.output_frequency_1_pin = 13;
  middleColor.color_selector_2_pin = 35;
  middleColor.color_selector_3_pin = 34;
  middleColor.out_pin = 33;
  middleColor.label = "Middle";
  middleColor.frequency = 20;
  middleColor.initialize();

 // ======

  gripperColor.output_frequency_0_pin = 14;
  gripperColor.output_frequency_1_pin = 13;
  gripperColor.color_selector_2_pin = 16;
  gripperColor.color_selector_3_pin = 15;
  gripperColor.out_pin = 12;
  gripperColor.label = "Gripper";
  gripperColor.frequency = 20;
  gripperColor.initialize();
}

void initColorCalibrations(){

  /*
  This is modified from the original code. Many of the calibration points
  were removed. See the charts of the calibration points for more points.
  */

  middleColor.addCalibrationPoint(BLACK, 359, 300, 407);
  rightColor.addCalibrationPoint(BLACK, 617, 672, 768);
  leftColor.addCalibrationPoint(BLACK, 521, 546, 591);
  middleColor.addCalibrationPoint(BLACK, 358, 297, 403);
  rightColor.addCalibrationPoint(BLACK, 662, 700, 783);

  rightColor.addCalibrationPoint(GREEN, 184, 232, 175);
  leftColor.addCalibrationPoint(GREEN, 202, 244, 175);
  rightColor.addCalibrationPoint(GREEN, 195, 255, 177);
  leftColor.addCalibrationPoint(GREEN, 207, 251, 173);
  rightColor.addCalibrationPoint(GREEN, 202, 269, 184);

  rightColor.addCalibrationPoint(BLUE, 628, 289, 469);
  leftColor.addCalibrationPoint(BLUE, 446, 175, 289);
  rightColor.addCalibrationPoint(BLUE, 703, 305, 513);
  middleColor.addCalibrationPoint(BLUE, 302, 136, 239);
  middleColor.addCalibrationPoint(BLUE, 326, 169, 278);

  rightColor.addCalibrationPoint(RED, 184, 409, 504);
  leftColor.addCalibrationPoint(RED, 157, 407, 496);
  rightColor.addCalibrationPoint(RED, 188, 424, 522);
  leftColor.addCalibrationPoint(RED, 152, 403, 489);

  middleColor.addCalibrationPoint(YELLOW, 126, 214, 196);
  middleColor.addCalibrationPoint(YELLOW, 125, 214, 195);
  leftColor.addCalibrationPoint(YELLOW, 105, 267, 188);
  leftColor.addCalibrationPoint(YELLOW, 106, 269, 190);
  rightColor.addCalibrationPoint(YELLOW, 140, 350, 258);
  rightColor.addCalibrationPoint(YELLOW, 142, 364, 264);

  gripperColor.addCalibrationPoint(RED, 140, 341, 389);
  gripperColor.addCalibrationPoint(RED, 112, 322, 382);
  gripperColor.addCalibrationPoint(BLUE, 402, 130, 248);
  gripperColor.addCalibrationPoint(BLUE, 412, 131, 252);
  gripperColor.addCalibrationPoint(BLUE, 442, 143, 273);

  rightColor.addCalibrationPoint(WHITE, 118, 114, 130);
  rightColor.addCalibrationPoint(WHITE, 112, 106, 122);
  middleColor.addCalibrationPoint(WHITE, 97, 78, 102);
  middleColor.addCalibrationPoint(WHITE, 104, 81, 108);
  leftColor.addCalibrationPoint(WHITE, 118, 119, 129);
  leftColor.addCalibrationPoint(WHITE, 117, 118, 128);
}
#endif