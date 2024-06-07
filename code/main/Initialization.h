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

  middleColor.addCalibrationPoint(BLACK, 359, 300, 407);
  rightColor.addCalibrationPoint(BLACK, 617, 672, 768);
  leftColor.addCalibrationPoint(BLACK, 521, 546, 591);
  middleColor.addCalibrationPoint(BLACK, 358, 297, 403);
  rightColor.addCalibrationPoint(BLACK, 662, 700, 783);
  leftColor.addCalibrationPoint(BLACK, 539, 579, 635);
  middleColor.addCalibrationPoint(BLACK, 362, 304, 405);
  rightColor.addCalibrationPoint(BLACK, 622, 660, 754);
  leftColor.addCalibrationPoint(BLACK, 496, 535, 583);
  middleColor.addCalibrationPoint(BLACK, 349, 308, 421);
  rightColor.addCalibrationPoint(BLACK, 674, 729, 839);
  leftColor.addCalibrationPoint(BLACK, 613, 656, 714);
  middleColor.addCalibrationPoint(BLACK, 493, 512, 722);
  rightColor.addCalibrationPoint(BLACK, 680, 722, 850);
  leftColor.addCalibrationPoint(BLACK, 514, 550, 603);
  middleColor.addCalibrationPoint(BLACK, 585, 496, 650);
  rightColor.addCalibrationPoint(BLACK, 596, 570, 638);
  leftColor.addCalibrationPoint(BLACK, 528, 582, 598);
  middleColor.addCalibrationPoint(BLACK, 434, 375, 482);
  rightColor.addCalibrationPoint(BLACK, 602, 594, 664);
  leftColor.addCalibrationPoint(BLACK, 529, 624, 677);
  middleColor.addCalibrationPoint(BLACK, 438, 363, 490);
  rightColor.addCalibrationPoint(BLACK, 605, 606, 721);
  leftColor.addCalibrationPoint(BLACK, 628, 700, 760);
  middleColor.addCalibrationPoint(BLACK, 348, 294, 390);
  rightColor.addCalibrationPoint(BLACK, 540, 514, 599);
  leftColor.addCalibrationPoint(BLACK, 546, 593, 639);
  middleColor.addCalibrationPoint(BLACK, 405, 339, 457);
  rightColor.addCalibrationPoint(BLACK, 696, 690, 823);
  leftColor.addCalibrationPoint(BLACK, 569, 650, 721);
  middleColor.addCalibrationPoint(BLACK, 478, 410, 550);
  rightColor.addCalibrationPoint(BLACK, 803, 794, 935);
  leftColor.addCalibrationPoint(BLACK, 542, 608, 641);
  middleColor.addCalibrationPoint(BLACK, 414, 350, 466);
  rightColor.addCalibrationPoint(BLACK, 631, 598, 704);
  leftColor.addCalibrationPoint(BLACK, 552, 602, 639);
  middleColor.addCalibrationPoint(BLACK, 427, 365, 491);
  rightColor.addCalibrationPoint(BLACK, 613, 590, 669);
  leftColor.addCalibrationPoint(BLACK, 541, 620, 649);
  middleColor.addCalibrationPoint(BLACK, 388, 316, 423);
  rightColor.addCalibrationPoint(BLACK, 595, 580, 681);
  leftColor.addCalibrationPoint(BLACK, 662, 746, 794);
  middleColor.addCalibrationPoint(BLACK, 379, 312, 420);
  rightColor.addCalibrationPoint(BLACK, 587, 579, 671);
  leftColor.addCalibrationPoint(BLACK, 563, 588, 647);
  middleColor.addCalibrationPoint(BLACK, 327, 268, 358);
  rightColor.addCalibrationPoint(BLACK, 418, 387, 458);
  leftColor.addCalibrationPoint(BLACK, 582, 679, 727);
  middleColor.addCalibrationPoint(BLACK, 389, 323, 454);
  rightColor.addCalibrationPoint(BLACK, 650, 678, 808);
  leftColor.addCalibrationPoint(BLACK, 551, 609, 642);
  middleColor.addCalibrationPoint(BLACK, 445, 362, 487);
  rightColor.addCalibrationPoint(BLACK, 706, 745, 848);
  leftColor.addCalibrationPoint(BLACK, 639, 705, 758);
  middleColor.addCalibrationPoint(BLACK, 391, 328, 455);
  rightColor.addCalibrationPoint(BLACK, 514, 558, 659);
  leftColor.addCalibrationPoint(BLACK, 644, 700, 740);
  middleColor.addCalibrationPoint(BLACK, 408, 336, 453);
  rightColor.addCalibrationPoint(BLACK, 662, 692, 814);
  leftColor.addCalibrationPoint(BLACK, 455, 500, 543);
  middleColor.addCalibrationPoint(BLACK, 386, 315, 422);
  rightColor.addCalibrationPoint(BLACK, 780, 770, 879);
  leftColor.addCalibrationPoint(BLACK, 359, 377, 414);
  middleColor.addCalibrationPoint(BLACK, 376, 313, 430);
  rightColor.addCalibrationPoint(BLACK, 572, 547, 604);
  leftColor.addCalibrationPoint(BLACK, 362, 384, 426);
  middleColor.addCalibrationPoint(BLACK, 335, 274, 376);
  rightColor.addCalibrationPoint(BLACK, 673, 676, 736);
  leftColor.addCalibrationPoint(BLACK, 562, 622, 699);
  middleColor.addCalibrationPoint(BLACK, 418, 342, 455);
  rightColor.addCalibrationPoint(BLACK, 638, 637, 699);
  leftColor.addCalibrationPoint(BLACK, 526, 599, 647);
  middleColor.addCalibrationPoint(BLACK, 399, 334, 451);
  rightColor.addCalibrationPoint(BLACK, 460, 471, 531);
  leftColor.addCalibrationPoint(BLACK, 476, 521, 555);
  middleColor.addCalibrationPoint(BLACK, 380, 309, 415);
  rightColor.addCalibrationPoint(BLACK, 770, 764, 929);
  leftColor.addCalibrationPoint(BLACK, 548, 612, 631);
  middleColor.addCalibrationPoint(BLACK, 415, 339, 448);
  rightColor.addCalibrationPoint(BLACK, 498, 493, 571);
  leftColor.addCalibrationPoint(BLACK, 479, 525, 577);
  middleColor.addCalibrationPoint(BLACK, 468, 390, 475);
  rightColor.addCalibrationPoint(BLACK, 729, 730, 835);
  leftColor.addCalibrationPoint(BLACK, 344, 355, 389);
  middleColor.addCalibrationPoint(BLACK, 494, 407, 553);
  rightColor.addCalibrationPoint(BLACK, 554, 537, 627);
  leftColor.addCalibrationPoint(BLACK, 560, 623, 676);
  middleColor.addCalibrationPoint(BLACK, 428, 356, 485);
  rightColor.addCalibrationPoint(BLACK, 577, 565, 655);
  leftColor.addCalibrationPoint(BLACK, 499, 532, 579);
  middleColor.addCalibrationPoint(BLACK, 354, 286, 386);
  rightColor.addCalibrationPoint(BLACK, 518, 503, 579);
  leftColor.addCalibrationPoint(BLACK, 456, 475, 527);
  middleColor.addCalibrationPoint(BLACK, 393, 320, 431);


  leftColor.addCalibrationPoint(BLACK, 540, 612, 663);
  middleColor.addCalibrationPoint(BLACK, 370, 305, 413);
  rightColor.addCalibrationPoint(BLACK, 594, 573, 654);
  leftColor.addCalibrationPoint(BLACK, 432, 493, 548);
  middleColor.addCalibrationPoint(BLACK, 408, 335, 453);
  rightColor.addCalibrationPoint(BLACK, 577, 563, 659);
  leftColor.addCalibrationPoint(BLACK, 406, 437, 470);
  middleColor.addCalibrationPoint(BLACK, 379, 311, 423);
  rightColor.addCalibrationPoint(BLACK, 549, 540, 642);
  leftColor.addCalibrationPoint(BLACK, 475, 520, 567);
  middleColor.addCalibrationPoint(BLACK, 413, 343, 455);
  rightColor.addCalibrationPoint(BLACK, 674, 631, 700);
  leftColor.addCalibrationPoint(BLACK, 555, 601, 647);
  middleColor.addCalibrationPoint(BLACK, 414, 334, 441);
  rightColor.addCalibrationPoint(BLACK, 545, 540, 661);
  leftColor.addCalibrationPoint(BLACK, 567, 597, 641);
  middleColor.addCalibrationPoint(BLACK, 370, 298, 394);
  rightColor.addCalibrationPoint(BLACK, 475, 439, 525);
  leftColor.addCalibrationPoint(BLACK, 595, 617, 688);
  middleColor.addCalibrationPoint(BLACK, 373, 307, 413);
  rightColor.addCalibrationPoint(BLACK, 559, 515, 586);
  leftColor.addCalibrationPoint(BLACK, 475, 496, 531);
  middleColor.addCalibrationPoint(BLACK, 345, 284, 390);
  rightColor.addCalibrationPoint(BLACK, 608, 556, 616);
  leftColor.addCalibrationPoint(BLACK, 297, 291, 318);
  middleColor.addCalibrationPoint(BLACK, 353, 285, 378);
  rightColor.addCalibrationPoint(BLACK, 632, 619, 736);
  leftColor.addCalibrationPoint(BLACK, 365, 372, 408);
  middleColor.addCalibrationPoint(BLACK, 403, 338, 457);
  rightColor.addCalibrationPoint(BLACK, 605, 581, 686);
  leftColor.addCalibrationPoint(BLACK, 607, 603, 666);
  middleColor.addCalibrationPoint(BLACK, 410, 335, 455);
  rightColor.addCalibrationPoint(BLACK, 817, 776, 923);
  leftColor.addCalibrationPoint(BLACK, 429, 449, 506);
  middleColor.addCalibrationPoint(BLACK, 405, 334, 458);
  rightColor.addCalibrationPoint(BLACK, 776, 730, 821);
  leftColor.addCalibrationPoint(BLACK, 446, 461, 505);
  middleColor.addCalibrationPoint(BLACK, 398, 319, 422);
  rightColor.addCalibrationPoint(BLACK, 633, 624, 713);
  leftColor.addCalibrationPoint(BLACK, 579, 626, 667);
  middleColor.addCalibrationPoint(BLACK, 369, 303, 413);
  rightColor.addCalibrationPoint(BLACK, 501, 508, 611);
  leftColor.addCalibrationPoint(BLACK, 374, 392, 435);
  middleColor.addCalibrationPoint(BLACK, 399, 316, 417);
  rightColor.addCalibrationPoint(BLACK, 429, 417, 500);
  leftColor.addCalibrationPoint(BLACK, 404, 423, 469);
  middleColor.addCalibrationPoint(BLACK, 337, 272, 366);
  rightColor.addCalibrationPoint(BLACK, 614, 589, 705);
  leftColor.addCalibrationPoint(BLACK, 382, 407, 439);
  middleColor.addCalibrationPoint(BLACK, 348, 284, 382);
  rightColor.addCalibrationPoint(BLACK, 690, 677, 799);
  leftColor.addCalibrationPoint(BLACK, 550, 597, 656);
  middleColor.addCalibrationPoint(BLACK, 373, 302, 402);
  rightColor.addCalibrationPoint(BLACK, 531, 506, 575);
  leftColor.addCalibrationPoint(BLACK, 472, 482, 528);
  middleColor.addCalibrationPoint(BLACK, 378, 307, 413);
  rightColor.addCalibrationPoint(BLACK, 499, 479, 565);
  leftColor.addCalibrationPoint(BLACK, 494, 545, 600);
  middleColor.addCalibrationPoint(BLACK, 391, 325, 446);
  rightColor.addCalibrationPoint(BLACK, 531, 497, 597);
  leftColor.addCalibrationPoint(BLACK, 435, 486, 514);
  middleColor.addCalibrationPoint(BLACK, 351, 289, 394);
  rightColor.addCalibrationPoint(BLACK, 506, 513, 621);
  leftColor.addCalibrationPoint(BLACK, 356, 358, 387);
  middleColor.addCalibrationPoint(BLACK, 416, 347, 468);
  rightColor.addCalibrationPoint(BLACK, 707, 693, 814);
  leftColor.addCalibrationPoint(BLACK, 408, 447, 486);
  middleColor.addCalibrationPoint(BLACK, 409, 335, 451);
  rightColor.addCalibrationPoint(BLACK, 603, 582, 686);
  leftColor.addCalibrationPoint(BLACK, 433, 468, 514);
  middleColor.addCalibrationPoint(BLACK, 410, 333, 457);
  rightColor.addCalibrationPoint(BLACK, 714, 693, 813);
  leftColor.addCalibrationPoint(BLACK, 445, 488, 524);
  middleColor.addCalibrationPoint(BLACK, 366, 295, 395);
  rightColor.addCalibrationPoint(BLACK, 516, 485, 526);
  leftColor.addCalibrationPoint(BLACK, 448, 518, 575);
  middleColor.addCalibrationPoint(BLACK, 378, 303, 403);
  rightColor.addCalibrationPoint(BLACK, 617, 537, 570);
  leftColor.addCalibrationPoint(BLACK, 476, 492, 521);
  middleColor.addCalibrationPoint(BLACK, 363, 301, 417);
  rightColor.addCalibrationPoint(BLACK, 753, 713, 821);
  leftColor.addCalibrationPoint(BLACK, 505, 577, 632);
  middleColor.addCalibrationPoint(BLACK, 373, 307, 402);
  rightColor.addCalibrationPoint(BLACK, 622, 680, 811);
  leftColor.addCalibrationPoint(BLACK, 508, 570, 628);
  middleColor.addCalibrationPoint(BLACK, 469, 402, 556);
  rightColor.addCalibrationPoint(BLACK, 663, 664, 794);
  leftColor.addCalibrationPoint(BLACK, 524, 593, 644);
  middleColor.addCalibrationPoint(BLACK, 419, 339, 456);
  rightColor.addCalibrationPoint(BLACK, 770, 747, 889);
  leftColor.addCalibrationPoint(BLACK, 385, 403, 439);
  middleColor.addCalibrationPoint(BLACK, 678, 565, 747);
  rightColor.addCalibrationPoint(BLACK, 835, 812, 961);
  leftColor.addCalibrationPoint(BLACK, 385, 404, 438);
  middleColor.addCalibrationPoint(BLACK, 622, 520, 696);


  rightColor.addCalibrationPoint(BLACK, 520, 519, 597);
  leftColor.addCalibrationPoint(BLACK, 395, 389, 433);
  middleColor.addCalibrationPoint(BLACK, 446, 367, 496);
  rightColor.addCalibrationPoint(BLACK, 418, 411, 479);
  leftColor.addCalibrationPoint(BLACK, 375, 368, 411);
  middleColor.addCalibrationPoint(BLACK, 405, 340, 449);
  rightColor.addCalibrationPoint(BLACK, 604, 639, 730);
  leftColor.addCalibrationPoint(BLACK, 489, 489, 526);
  middleColor.addCalibrationPoint(BLACK, 592, 503, 674);
  rightColor.addCalibrationPoint(BLACK, 602, 630, 712);
  leftColor.addCalibrationPoint(BLACK, 608, 612, 657);
  middleColor.addCalibrationPoint(BLACK, 416, 340, 455);
  rightColor.addCalibrationPoint(BLACK, 549, 564, 639);
  leftColor.addCalibrationPoint(BLACK, 457, 426, 447);
  middleColor.addCalibrationPoint(BLACK, 342, 291, 411);
  rightColor.addCalibrationPoint(BLACK, 569, 606, 688);
  leftColor.addCalibrationPoint(BLACK, 552, 569, 617);
  middleColor.addCalibrationPoint(BLACK, 397, 327, 441);
  rightColor.addCalibrationPoint(BLACK, 620, 680, 777);
  leftColor.addCalibrationPoint(BLACK, 594, 598, 642);
  middleColor.addCalibrationPoint(BLACK, 352, 288, 383);
  rightColor.addCalibrationPoint(BLACK, 569, 595, 682);
  leftColor.addCalibrationPoint(BLACK, 452, 453, 499);
  middleColor.addCalibrationPoint(BLACK, 392, 323, 440);
  rightColor.addCalibrationPoint(BLACK, 537, 556, 620);
  leftColor.addCalibrationPoint(BLACK, 493, 498, 544);
  middleColor.addCalibrationPoint(BLACK, 740, 493, 675);
  rightColor.addCalibrationPoint(BLACK, 513, 509, 565);
  leftColor.addCalibrationPoint(BLACK, 537, 545, 592);
  middleColor.addCalibrationPoint(BLACK, 580, 427, 543);
  rightColor.addCalibrationPoint(BLACK, 584, 657, 753);
  leftColor.addCalibrationPoint(BLACK, 592, 607, 687);
  middleColor.addCalibrationPoint(BLACK, 577, 434, 519);
  rightColor.addCalibrationPoint(BLACK, 616, 678, 762);
  leftColor.addCalibrationPoint(BLACK, 451, 486, 571);
  middleColor.addCalibrationPoint(BLACK, 398, 332, 444);
  rightColor.addCalibrationPoint(BLACK, 410, 452, 500);
  leftColor.addCalibrationPoint(BLACK, 603, 621, 685);
  middleColor.addCalibrationPoint(BLACK, 411, 340, 459);
  rightColor.addCalibrationPoint(BLACK, 593, 676, 778);
  leftColor.addCalibrationPoint(BLACK, 497, 495, 528);
  middleColor.addCalibrationPoint(BLACK, 360, 294, 389);
  rightColor.addCalibrationPoint(BLACK, 521, 565, 634);
  leftColor.addCalibrationPoint(BLACK, 607, 617, 663);
  middleColor.addCalibrationPoint(BLACK, 408, 335, 461);
  rightColor.addCalibrationPoint(BLACK, 579, 623, 691);
  leftColor.addCalibrationPoint(BLACK, 623, 642, 709);
  middleColor.addCalibrationPoint(BLACK, 470, 383, 489);
  rightColor.addCalibrationPoint(BLACK, 558, 575, 629);
  leftColor.addCalibrationPoint(BLACK, 461, 452, 477);
  middleColor.addCalibrationPoint(BLACK, 458, 382, 510);
  rightColor.addCalibrationPoint(BLACK, 504, 518, 586);
  leftColor.addCalibrationPoint(BLACK, 681, 700, 766);
  middleColor.addCalibrationPoint(BLACK, 459, 383, 513);
  rightColor.addCalibrationPoint(BLACK, 665, 691, 764);
  leftColor.addCalibrationPoint(BLACK, 585, 612, 689);
  middleColor.addCalibrationPoint(BLACK, 406, 350, 468);
  rightColor.addCalibrationPoint(BLACK, 503, 547, 721);
  leftColor.addCalibrationPoint(BLACK, 456, 468, 574);
  middleColor.addCalibrationPoint(BLACK, 400, 324, 445);
  rightColor.addCalibrationPoint(BLACK, 476, 465, 494);
  leftColor.addCalibrationPoint(BLACK, 502, 506, 552);
  middleColor.addCalibrationPoint(BLACK, 347, 282, 375);
  rightColor.addCalibrationPoint(BLACK, 559, 595, 667);
  leftColor.addCalibrationPoint(BLACK, 435, 451, 524);
  middleColor.addCalibrationPoint(BLACK, 342, 280, 376);
  rightColor.addCalibrationPoint(BLACK, 451, 465, 526);
  leftColor.addCalibrationPoint(BLACK, 441, 428, 469);
  middleColor.addCalibrationPoint(BLACK, 373, 308, 412);
  rightColor.addCalibrationPoint(BLACK, 495, 512, 587);
  leftColor.addCalibrationPoint(BLACK, 598, 617, 671);
  middleColor.addCalibrationPoint(BLACK, 397, 331, 446);
  rightColor.addCalibrationPoint(BLACK, 641, 670, 766);
  leftColor.addCalibrationPoint(BLACK, 668, 693, 775);
  middleColor.addCalibrationPoint(BLACK, 391, 320, 430);
  rightColor.addCalibrationPoint(BLACK, 522, 555, 632);
  leftColor.addCalibrationPoint(BLACK, 445, 469, 527);
  middleColor.addCalibrationPoint(BLACK, 344, 282, 381);
  rightColor.addCalibrationPoint(BLACK, 545, 572, 647);
  leftColor.addCalibrationPoint(BLACK, 452, 462, 506);
  middleColor.addCalibrationPoint(BLACK, 404, 336, 459);
  rightColor.addCalibrationPoint(BLACK, 608, 649, 733);
  leftColor.addCalibrationPoint(BLACK, 484, 497, 542);
  middleColor.addCalibrationPoint(BLACK, 394, 325, 437);

  rightColor.addCalibrationPoint(BLACK, 715, 753, 865);
  leftColor.addCalibrationPoint(BLACK, 570, 597, 655);
  middleColor.addCalibrationPoint(BLACK, 424, 348, 472);
  rightColor.addCalibrationPoint(BLACK, 713, 750, 860);
  leftColor.addCalibrationPoint(BLACK, 558, 584, 638);
  middleColor.addCalibrationPoint(BLACK, 423, 347, 471);
  rightColor.addCalibrationPoint(BLACK, 709, 748, 859);
  leftColor.addCalibrationPoint(BLACK, 559, 584, 637);
  middleColor.addCalibrationPoint(BLACK, 423, 347, 470);
  rightColor.addCalibrationPoint(BLACK, 710, 749, 858);
  leftColor.addCalibrationPoint(BLACK, 550, 575, 626);
  middleColor.addCalibrationPoint(BLACK, 423, 347, 471);
  rightColor.addCalibrationPoint(BLACK, 709, 747, 856);
  leftColor.addCalibrationPoint(BLACK, 543, 568, 618);
  middleColor.addCalibrationPoint(BLACK, 423, 347, 470);
  rightColor.addCalibrationPoint(BLACK, 576, 600, 690);
  leftColor.addCalibrationPoint(BLACK, 529, 552, 605);
  middleColor.addCalibrationPoint(BLACK, 410, 339, 454);
  rightColor.addCalibrationPoint(BLACK, 563, 573, 647);
  leftColor.addCalibrationPoint(BLACK, 473, 478, 522);
  middleColor.addCalibrationPoint(BLACK, 397, 333, 456);
  rightColor.addCalibrationPoint(BLACK, 533, 566, 662);
  leftColor.addCalibrationPoint(BLACK, 465, 459, 496);
  middleColor.addCalibrationPoint(BLACK, 379, 310, 408);
  rightColor.addCalibrationPoint(BLACK, 500, 521, 615);
  leftColor.addCalibrationPoint(BLACK, 518, 540, 603);
  middleColor.addCalibrationPoint(BLACK, 369, 324, 441);
  rightColor.addCalibrationPoint(BLACK, 522, 557, 656);
  leftColor.addCalibrationPoint(BLACK, 472, 504, 574);
  middleColor.addCalibrationPoint(BLACK, 413, 334, 450);
  rightColor.addCalibrationPoint(BLACK, 550, 564, 644);
  leftColor.addCalibrationPoint(BLACK, 479, 509, 568);
  middleColor.addCalibrationPoint(BLACK, 392, 316, 429);
  rightColor.addCalibrationPoint(BLACK, 548, 568, 649);
  leftColor.addCalibrationPoint(BLACK, 407, 422, 463);
  middleColor.addCalibrationPoint(BLACK, 417, 337, 460);
  rightColor.addCalibrationPoint(BLACK, 409, 408, 469);
  leftColor.addCalibrationPoint(BLACK, 383, 397, 438);
  middleColor.addCalibrationPoint(BLACK, 437, 352, 475);
  rightColor.addCalibrationPoint(BLACK, 656, 676, 771);
  leftColor.addCalibrationPoint(BLACK, 432, 444, 496);
  middleColor.addCalibrationPoint(BLACK, 374, 301, 408);
  rightColor.addCalibrationPoint(BLACK, 656, 673, 774);
  leftColor.addCalibrationPoint(BLACK, 416, 423, 465);
  middleColor.addCalibrationPoint(BLACK, 375, 304, 409);
  rightColor.addCalibrationPoint(BLACK, 651, 667, 767);
  leftColor.addCalibrationPoint(BLACK, 406, 418, 459);
  middleColor.addCalibrationPoint(BLACK, 369, 301, 407);
  rightColor.addCalibrationPoint(BLACK, 656, 672, 771);
  leftColor.addCalibrationPoint(BLACK, 400, 415, 455);
  middleColor.addCalibrationPoint(BLACK, 365, 300, 403);
  rightColor.addCalibrationPoint(BLACK, 662, 676, 777);
  leftColor.addCalibrationPoint(BLACK, 400, 415, 455);
  middleColor.addCalibrationPoint(BLACK, 365, 299, 402);

  rightColor.addCalibrationPoint(BLACK, 508, 564, 639);
  leftColor.addCalibrationPoint(BLACK, 529, 574, 619);
  middleColor.addCalibrationPoint(BLACK, 317, 266, 353);
  rightColor.addCalibrationPoint(BLACK, 524, 562, 632);
  leftColor.addCalibrationPoint(BLACK, 403, 424, 462);
  middleColor.addCalibrationPoint(BLACK, 468, 395, 539);
  rightColor.addCalibrationPoint(BLACK, 484, 512, 594);
  leftColor.addCalibrationPoint(BLACK, 466, 505, 541);
  middleColor.addCalibrationPoint(BLACK, 392, 334, 449);
  rightColor.addCalibrationPoint(BLACK, 571, 590, 646);
  leftColor.addCalibrationPoint(BLACK, 471, 518, 566);
  middleColor.addCalibrationPoint(BLACK, 424, 356, 480);
  rightColor.addCalibrationPoint(BLACK, 491, 521, 589);
  leftColor.addCalibrationPoint(BLACK, 539, 630, 652);
  middleColor.addCalibrationPoint(BLACK, 433, 369, 491);
  rightColor.addCalibrationPoint(BLACK, 428, 428, 486);
  leftColor.addCalibrationPoint(BLACK, 507, 527, 587);
  middleColor.addCalibrationPoint(BLACK, 369, 312, 421);
  rightColor.addCalibrationPoint(BLACK, 408, 417, 458);
  leftColor.addCalibrationPoint(BLACK, 473, 511, 546);
  middleColor.addCalibrationPoint(BLACK, 388, 327, 439);
  rightColor.addCalibrationPoint(BLACK, 582, 604, 675);
  leftColor.addCalibrationPoint(BLACK, 393, 424, 494);
  middleColor.addCalibrationPoint(BLACK, 382, 319, 436);
  rightColor.addCalibrationPoint(BLACK, 439, 442, 516);
  leftColor.addCalibrationPoint(BLACK, 436, 499, 535);
  middleColor.addCalibrationPoint(BLACK, 370, 304, 403);
  rightColor.addCalibrationPoint(BLACK, 554, 589, 659);
  leftColor.addCalibrationPoint(BLACK, 451, 505, 569);
  middleColor.addCalibrationPoint(BLACK, 298, 247, 335);
  rightColor.addCalibrationPoint(BLACK, 657, 723, 807);
  leftColor.addCalibrationPoint(BLACK, 435, 473, 536);
  middleColor.addCalibrationPoint(BLACK, 383, 326, 429);
  rightColor.addCalibrationPoint(BLACK, 448, 494, 563);
  leftColor.addCalibrationPoint(BLACK, 476, 495, 519);
  middleColor.addCalibrationPoint(BLACK, 367, 310, 425);
  rightColor.addCalibrationPoint(BLACK, 503, 538, 594);
  leftColor.addCalibrationPoint(BLACK, 388, 417, 447);
  middleColor.addCalibrationPoint(BLACK, 371, 340, 467);
  rightColor.addCalibrationPoint(BLACK, 509, 554, 609);
  leftColor.addCalibrationPoint(BLACK, 445, 483, 526);
  middleColor.addCalibrationPoint(BLACK, 385, 324, 435);
  rightColor.addCalibrationPoint(BLACK, 507, 551, 605);
  leftColor.addCalibrationPoint(BLACK, 430, 467, 507);
  middleColor.addCalibrationPoint(BLACK, 386, 324, 436);
  rightColor.addCalibrationPoint(BLACK, 501, 547, 601);
  leftColor.addCalibrationPoint(BLACK, 428, 466, 506);
  middleColor.addCalibrationPoint(BLACK, 371, 311, 417);

  rightColor.addCalibrationPoint(GREEN, 184, 232, 175);
  leftColor.addCalibrationPoint(GREEN, 202, 244, 175);
  rightColor.addCalibrationPoint(GREEN, 195, 255, 177);
  leftColor.addCalibrationPoint(GREEN, 207, 251, 173);
  rightColor.addCalibrationPoint(GREEN, 202, 269, 184);
  leftColor.addCalibrationPoint(GREEN, 187, 239, 165);
  rightColor.addCalibrationPoint(GREEN, 198, 266, 182);
  leftColor.addCalibrationPoint(GREEN, 198, 237, 165);
  rightColor.addCalibrationPoint(GREEN, 189, 245, 179);
  leftColor.addCalibrationPoint(GREEN, 173, 207, 135);
  rightColor.addCalibrationPoint(GREEN, 205, 276, 197);
  leftColor.addCalibrationPoint(GREEN, 175, 203, 148);
  rightColor.addCalibrationPoint(GREEN, 189, 268, 190);
  leftColor.addCalibrationPoint(GREEN, 206, 249, 169);
  rightColor.addCalibrationPoint(GREEN, 206, 303, 214);
  leftColor.addCalibrationPoint(GREEN, 169, 197, 137);
  rightColor.addCalibrationPoint(GREEN, 205, 276, 196);
  leftColor.addCalibrationPoint(GREEN, 176, 208, 146);
  rightColor.addCalibrationPoint(GREEN, 202, 266, 189);
  leftColor.addCalibrationPoint(GREEN, 218, 261, 181);
  rightColor.addCalibrationPoint(GREEN, 212, 282, 196);
  leftColor.addCalibrationPoint(GREEN, 215, 257, 178);
  rightColor.addCalibrationPoint(GREEN, 204, 266, 192);
  leftColor.addCalibrationPoint(GREEN, 195, 231, 159);
  rightColor.addCalibrationPoint(GREEN, 193, 244, 183);
  leftColor.addCalibrationPoint(GREEN, 200, 234, 172);
  rightColor.addCalibrationPoint(GREEN, 235, 307, 218);
  leftColor.addCalibrationPoint(GREEN, 190, 224, 155);
  rightColor.addCalibrationPoint(GREEN, 216, 267, 198);
  leftColor.addCalibrationPoint(GREEN, 198, 233, 166);
  rightColor.addCalibrationPoint(GREEN, 211, 256, 192);
  leftColor.addCalibrationPoint(GREEN, 210, 248, 171);
  rightColor.addCalibrationPoint(GREEN, 233, 294, 209);
  leftColor.addCalibrationPoint(GREEN, 197, 235, 166);
  rightColor.addCalibrationPoint(GREEN, 215, 262, 197);
  leftColor.addCalibrationPoint(GREEN, 202, 244, 162);
  rightColor.addCalibrationPoint(GREEN, 226, 280, 206);
  leftColor.addCalibrationPoint(GREEN, 173, 205, 139);
  rightColor.addCalibrationPoint(GREEN, 227, 279, 209);
  leftColor.addCalibrationPoint(GREEN, 165, 188, 141);
  rightColor.addCalibrationPoint(GREEN, 225, 277, 206);
  leftColor.addCalibrationPoint(GREEN, 200, 233, 166);
  rightColor.addCalibrationPoint(GREEN, 229, 289, 206);
  leftColor.addCalibrationPoint(GREEN, 197, 234, 161);
  rightColor.addCalibrationPoint(GREEN, 226, 282, 203);
  leftColor.addCalibrationPoint(GREEN, 212, 249, 182);
  rightColor.addCalibrationPoint(GREEN, 221, 273, 196);
  leftColor.addCalibrationPoint(GREEN, 237, 276, 208);
  rightColor.addCalibrationPoint(GREEN, 228, 284, 204);
  leftColor.addCalibrationPoint(GREEN, 244, 300, 212);
  rightColor.addCalibrationPoint(GREEN, 234, 288, 213);
  leftColor.addCalibrationPoint(GREEN, 221, 259, 192);
  rightColor.addCalibrationPoint(GREEN, 234, 295, 209);
  leftColor.addCalibrationPoint(GREEN, 232, 277, 199);
  rightColor.addCalibrationPoint(GREEN, 218, 257, 191);
  leftColor.addCalibrationPoint(GREEN, 252, 304, 219);
  rightColor.addCalibrationPoint(GREEN, 234, 291, 209);
  leftColor.addCalibrationPoint(GREEN, 220, 260, 194);
  rightColor.addCalibrationPoint(GREEN, 214, 265, 187);
  leftColor.addCalibrationPoint(GREEN, 227, 270, 202);
  rightColor.addCalibrationPoint(GREEN, 220, 273, 192);
  leftColor.addCalibrationPoint(GREEN, 228, 272, 204);

  middleColor.addCalibrationPoint(GREEN, 162, 167, 144);
  middleColor.addCalibrationPoint(GREEN, 163, 166, 143);
  middleColor.addCalibrationPoint(GREEN, 163, 167, 144);
  middleColor.addCalibrationPoint(GREEN, 177, 182, 159);
  middleColor.addCalibrationPoint(GREEN, 166, 165, 148);
  middleColor.addCalibrationPoint(GREEN, 172, 177, 152);
  middleColor.addCalibrationPoint(GREEN, 171, 172, 154);
  middleColor.addCalibrationPoint(GREEN, 159, 160, 140);
  middleColor.addCalibrationPoint(GREEN, 169, 175, 147);
  middleColor.addCalibrationPoint(GREEN, 191, 197, 174);
  middleColor.addCalibrationPoint(GREEN, 174, 176, 155);
  middleColor.addCalibrationPoint(GREEN, 192, 198, 173);
  middleColor.addCalibrationPoint(GREEN, 177, 179, 157);
  middleColor.addCalibrationPoint(GREEN, 178, 179, 159);
  middleColor.addCalibrationPoint(GREEN, 172, 177, 154);
  middleColor.addCalibrationPoint(GREEN, 178, 182, 159);
  middleColor.addCalibrationPoint(GREEN, 176, 179, 162);
  middleColor.addCalibrationPoint(GREEN, 178, 183, 161);
  middleColor.addCalibrationPoint(GREEN, 170, 174, 152);
  middleColor.addCalibrationPoint(GREEN, 169, 173, 151);
  middleColor.addCalibrationPoint(GREEN, 171, 174, 149);
  middleColor.addCalibrationPoint(GREEN, 173, 173, 157);
  middleColor.addCalibrationPoint(GREEN, 168, 175, 143);
  middleColor.addCalibrationPoint(GREEN, 162, 171, 139);
  middleColor.addCalibrationPoint(GREEN, 165, 171, 144);
  middleColor.addCalibrationPoint(GREEN, 181, 189, 160);
  middleColor.addCalibrationPoint(GREEN, 183, 186, 161);
  middleColor.addCalibrationPoint(GREEN, 201, 212, 169);
  middleColor.addCalibrationPoint(GREEN, 177, 182, 157);
  middleColor.addCalibrationPoint(GREEN, 181, 187, 159);
  middleColor.addCalibrationPoint(GREEN, 185, 189, 164);
  middleColor.addCalibrationPoint(GREEN, 199, 206, 175);
  middleColor.addCalibrationPoint(GREEN, 203, 217, 187);
  middleColor.addCalibrationPoint(GREEN, 197, 220, 188);
  middleColor.addCalibrationPoint(GREEN, 223, 233, 202);

  middleColor.addCalibrationPoint(GREEN, 173, 175, 151);
  middleColor.addCalibrationPoint(GREEN, 171, 172, 148);
  middleColor.addCalibrationPoint(GREEN, 175, 194, 178);
  middleColor.addCalibrationPoint(GREEN, 193, 196, 174);
  middleColor.addCalibrationPoint(GREEN, 197, 195, 178);
  middleColor.addCalibrationPoint(GREEN, 179, 179, 165);
  middleColor.addCalibrationPoint(GREEN, 191, 191, 172);
  middleColor.addCalibrationPoint(GREEN, 191, 189, 170);
  middleColor.addCalibrationPoint(GREEN, 186, 185, 167);
  middleColor.addCalibrationPoint(GREEN, 190, 188, 171);
  middleColor.addCalibrationPoint(GREEN, 195, 192, 174);
  middleColor.addCalibrationPoint(GREEN, 196, 194, 174);
  middleColor.addCalibrationPoint(GREEN, 187, 186, 168);
  middleColor.addCalibrationPoint(GREEN, 185, 181, 166);
  middleColor.addCalibrationPoint(GREEN, 189, 187, 168);
  middleColor.addCalibrationPoint(GREEN, 175, 175, 156);
  middleColor.addCalibrationPoint(GREEN, 188, 186, 167);
  middleColor.addCalibrationPoint(GREEN, 168, 168, 151);
  middleColor.addCalibrationPoint(GREEN, 187, 187, 169);
  middleColor.addCalibrationPoint(GREEN, 176, 176, 155);
  middleColor.addCalibrationPoint(GREEN, 184, 186, 162);
  middleColor.addCalibrationPoint(GREEN, 182, 183, 161);

  rightColor.addCalibrationPoint(BLUE, 628, 289, 469);
  leftColor.addCalibrationPoint(BLUE, 446, 175, 289);
  rightColor.addCalibrationPoint(BLUE, 703, 305, 513);
  leftColor.addCalibrationPoint(BLUE, 415, 182, 288);
  rightColor.addCalibrationPoint(BLUE, 624, 211, 388);
  leftColor.addCalibrationPoint(BLUE, 392, 187, 287);
  rightColor.addCalibrationPoint(BLUE, 597, 222, 399);
  leftColor.addCalibrationPoint(BLUE, 383, 190, 290);
  rightColor.addCalibrationPoint(BLUE, 643, 294, 486);
  leftColor.addCalibrationPoint(BLUE, 445, 207, 321);
  rightColor.addCalibrationPoint(BLUE, 644, 274, 465);
  leftColor.addCalibrationPoint(BLUE, 435, 203, 315);
  rightColor.addCalibrationPoint(BLUE, 531, 253, 405);
  leftColor.addCalibrationPoint(BLUE, 471, 173, 294);
  rightColor.addCalibrationPoint(BLUE, 625, 267, 455);
  leftColor.addCalibrationPoint(BLUE, 425, 193, 304);
  rightColor.addCalibrationPoint(BLUE, 574, 271, 435);
  leftColor.addCalibrationPoint(BLUE, 493, 208, 333);
  rightColor.addCalibrationPoint(BLUE, 535, 260, 423);
  leftColor.addCalibrationPoint(BLUE, 466, 187, 306);
  rightColor.addCalibrationPoint(BLUE, 680, 273, 474);
  leftColor.addCalibrationPoint(BLUE, 463, 197, 317);
  rightColor.addCalibrationPoint(BLUE, 606, 274, 456);
  leftColor.addCalibrationPoint(BLUE, 474, 213, 335);
  rightColor.addCalibrationPoint(BLUE, 666, 293, 488);
  leftColor.addCalibrationPoint(BLUE, 483, 215, 340);
  rightColor.addCalibrationPoint(BLUE, 509, 248, 396);
  leftColor.addCalibrationPoint(BLUE, 410, 197, 304);
  rightColor.addCalibrationPoint(BLUE, 682, 297, 500);
  leftColor.addCalibrationPoint(BLUE, 477, 204, 329);
  rightColor.addCalibrationPoint(BLUE, 596, 268, 448);
  leftColor.addCalibrationPoint(BLUE, 364, 172, 266);
  rightColor.addCalibrationPoint(BLUE, 586, 264, 441);
  leftColor.addCalibrationPoint(BLUE, 362, 173, 266);
  rightColor.addCalibrationPoint(BLUE, 579, 262, 437);
  leftColor.addCalibrationPoint(BLUE, 368, 175, 269);
  rightColor.addCalibrationPoint(BLUE, 576, 257, 430);
  leftColor.addCalibrationPoint(BLUE, 435, 186, 300);

  rightColor.addCalibrationPoint(BLUE, 472, 219, 358);
  leftColor.addCalibrationPoint(BLUE, 388, 162, 263);
  rightColor.addCalibrationPoint(BLUE, 471, 218, 357);
  leftColor.addCalibrationPoint(BLUE, 388, 161, 262);
  rightColor.addCalibrationPoint(BLUE, 479, 221, 361);
  leftColor.addCalibrationPoint(BLUE, 389, 162, 265);
  rightColor.addCalibrationPoint(BLUE, 537, 230, 390);
  leftColor.addCalibrationPoint(BLUE, 419, 175, 285);
  rightColor.addCalibrationPoint(BLUE, 596, 248, 433);
  leftColor.addCalibrationPoint(BLUE, 444, 203, 321);
  rightColor.addCalibrationPoint(BLUE, 474, 259, 394);
  leftColor.addCalibrationPoint(BLUE, 456, 208, 329);
  rightColor.addCalibrationPoint(BLUE, 516, 259, 409);
  leftColor.addCalibrationPoint(BLUE, 387, 194, 295);
  rightColor.addCalibrationPoint(BLUE, 634, 278, 466);
  leftColor.addCalibrationPoint(BLUE, 505, 212, 346);
  rightColor.addCalibrationPoint(BLUE, 593, 260, 433);
  leftColor.addCalibrationPoint(BLUE, 454, 212, 331);
  rightColor.addCalibrationPoint(BLUE, 483, 226, 367);
  leftColor.addCalibrationPoint(BLUE, 470, 207, 331);
  rightColor.addCalibrationPoint(BLUE, 485, 230, 370);
  leftColor.addCalibrationPoint(BLUE, 516, 219, 355);
  rightColor.addCalibrationPoint(BLUE, 454, 212, 345);
  leftColor.addCalibrationPoint(BLUE, 488, 199, 327);

  middleColor.addCalibrationPoint(BLUE, 353, 160, 280);
  middleColor.addCalibrationPoint(BLUE, 354, 185, 303);
  middleColor.addCalibrationPoint(BLUE, 307, 157, 260);
  middleColor.addCalibrationPoint(BLUE, 319, 158, 264);
  middleColor.addCalibrationPoint(BLUE, 284, 141, 237);
  middleColor.addCalibrationPoint(BLUE, 317, 145, 253);
  middleColor.addCalibrationPoint(BLUE, 358, 166, 282);
  middleColor.addCalibrationPoint(BLUE, 308, 141, 245);
  middleColor.addCalibrationPoint(BLUE, 328, 154, 263);
  middleColor.addCalibrationPoint(BLUE, 316, 147, 257);
  middleColor.addCalibrationPoint(BLUE, 377, 191, 309);
  middleColor.addCalibrationPoint(BLUE, 326, 151, 263);
  middleColor.addCalibrationPoint(BLUE, 334, 156, 270);
  middleColor.addCalibrationPoint(BLUE, 321, 147, 257);
  middleColor.addCalibrationPoint(BLUE, 352, 157, 278);
  middleColor.addCalibrationPoint(BLUE, 327, 146, 259);

  middleColor.addCalibrationPoint(BLUE, 311, 140, 248);
  middleColor.addCalibrationPoint(BLUE, 302, 136, 239);
  middleColor.addCalibrationPoint(BLUE, 326, 169, 278);
  middleColor.addCalibrationPoint(BLUE, 331, 173, 284);
  middleColor.addCalibrationPoint(BLUE, 343, 178, 294);
  middleColor.addCalibrationPoint(BLUE, 303, 154, 258);
  middleColor.addCalibrationPoint(BLUE, 295, 155, 256);
  middleColor.addCalibrationPoint(BLUE, 324, 169, 281);
  middleColor.addCalibrationPoint(BLUE, 303, 150, 253);
  middleColor.addCalibrationPoint(BLUE, 327, 152, 266);
  middleColor.addCalibrationPoint(BLUE, 324, 143, 256);

  middleColor.addCalibrationPoint(BLUE, 304, 124, 230);
  middleColor.addCalibrationPoint(BLUE, 304, 126, 231);
  middleColor.addCalibrationPoint(BLUE, 317, 123, 233);
  middleColor.addCalibrationPoint(BLUE, 292, 124, 225);
  middleColor.addCalibrationPoint(BLUE, 295, 137, 238);
  middleColor.addCalibrationPoint(BLUE, 295, 136, 239);
  middleColor.addCalibrationPoint(BLUE, 302, 135, 240);
  middleColor.addCalibrationPoint(BLUE, 301, 142, 246);
  middleColor.addCalibrationPoint(BLUE, 292, 136, 237);
  middleColor.addCalibrationPoint(BLUE, 284, 139, 237);
  middleColor.addCalibrationPoint(BLUE, 280, 133, 228);
  middleColor.addCalibrationPoint(BLUE, 299, 153, 254);
  middleColor.addCalibrationPoint(BLUE, 296, 143, 244);
  middleColor.addCalibrationPoint(BLUE, 341, 160, 283);
  middleColor.addCalibrationPoint(BLUE, 317, 143, 256);
  middleColor.addCalibrationPoint(BLUE, 323, 143, 260);
  middleColor.addCalibrationPoint(BLUE, 320, 146, 261);
  middleColor.addCalibrationPoint(BLUE, 308, 141, 250);
  middleColor.addCalibrationPoint(BLUE, 305, 148, 257);
  middleColor.addCalibrationPoint(BLUE, 303, 141, 248);
  middleColor.addCalibrationPoint(BLUE, 305, 143, 253);

  middleColor.addCalibrationPoint(RED, 178, 275, 364);
  middleColor.addCalibrationPoint(RED, 179, 274, 363);
  middleColor.addCalibrationPoint(RED, 178, 270, 358);
  middleColor.addCalibrationPoint(RED, 202, 292, 387);
  middleColor.addCalibrationPoint(RED, 200, 299, 402);
  middleColor.addCalibrationPoint(RED, 193, 291, 389);
  middleColor.addCalibrationPoint(RED, 195, 296, 401);
  middleColor.addCalibrationPoint(RED, 190, 286, 381);
  middleColor.addCalibrationPoint(RED, 190, 300, 403);
  middleColor.addCalibrationPoint(RED, 208, 305, 408);
  middleColor.addCalibrationPoint(RED, 197, 299, 400);
  middleColor.addCalibrationPoint(RED, 185, 289, 385);
  middleColor.addCalibrationPoint(RED, 182, 274, 362);
  middleColor.addCalibrationPoint(RED, 183, 287, 384);
  middleColor.addCalibrationPoint(RED, 174, 289, 392);
  middleColor.addCalibrationPoint(RED, 178, 270, 359);
  middleColor.addCalibrationPoint(RED, 157, 271, 366);
  middleColor.addCalibrationPoint(RED, 162, 273, 361);
  middleColor.addCalibrationPoint(RED, 155, 259, 342);
  middleColor.addCalibrationPoint(RED, 167, 274, 361);
  middleColor.addCalibrationPoint(RED, 169, 269, 351);
  middleColor.addCalibrationPoint(RED, 172, 277, 363);
  middleColor.addCalibrationPoint(RED, 168, 268, 350);
  middleColor.addCalibrationPoint(RED, 154, 255, 337);
  middleColor.addCalibrationPoint(RED, 159, 262, 347);
  middleColor.addCalibrationPoint(RED, 151, 241, 319);
  middleColor.addCalibrationPoint(RED, 159, 251, 334);
  middleColor.addCalibrationPoint(RED, 155, 268, 367);
  middleColor.addCalibrationPoint(RED, 180, 264, 347);
  middleColor.addCalibrationPoint(RED, 164, 270, 363);
  middleColor.addCalibrationPoint(RED, 153, 234, 312);
  middleColor.addCalibrationPoint(RED, 137, 248, 336);
  middleColor.addCalibrationPoint(RED, 158, 246, 326);
  middleColor.addCalibrationPoint(RED, 142, 240, 323);
  middleColor.addCalibrationPoint(RED, 165, 255, 337);
  middleColor.addCalibrationPoint(RED, 164, 237, 313);
  middleColor.addCalibrationPoint(RED, 171, 249, 333);
  middleColor.addCalibrationPoint(RED, 170, 267, 357);
  middleColor.addCalibrationPoint(RED, 162, 249, 329);
  middleColor.addCalibrationPoint(RED, 154, 233, 311);
  middleColor.addCalibrationPoint(RED, 159, 267, 358);
  middleColor.addCalibrationPoint(RED, 157, 247, 330);
  middleColor.addCalibrationPoint(RED, 154, 248, 333);

  rightColor.addCalibrationPoint(RED, 184, 409, 504);
  leftColor.addCalibrationPoint(RED, 157, 407, 496);
  rightColor.addCalibrationPoint(RED, 188, 424, 522);
  leftColor.addCalibrationPoint(RED, 152, 403, 489);
  rightColor.addCalibrationPoint(RED, 189, 406, 494);
  leftColor.addCalibrationPoint(RED, 147, 391, 478);
  rightColor.addCalibrationPoint(RED, 207, 490, 602);
  leftColor.addCalibrationPoint(RED, 164, 432, 524);
  rightColor.addCalibrationPoint(RED, 186, 494, 640);
  leftColor.addCalibrationPoint(RED, 170, 444, 534);
  rightColor.addCalibrationPoint(RED, 194, 453, 551);
  leftColor.addCalibrationPoint(RED, 151, 376, 457);
  rightColor.addCalibrationPoint(RED, 208, 459, 563);
  leftColor.addCalibrationPoint(RED, 145, 360, 436);
  rightColor.addCalibrationPoint(RED, 205, 490, 604);
  leftColor.addCalibrationPoint(RED, 156, 345, 404);
  rightColor.addCalibrationPoint(RED, 232, 549, 684);
  leftColor.addCalibrationPoint(RED, 128, 366, 462);
  rightColor.addCalibrationPoint(RED, 223, 534, 662);
  leftColor.addCalibrationPoint(RED, 138, 378, 471);
  rightColor.addCalibrationPoint(RED, 205, 492, 609);
  leftColor.addCalibrationPoint(RED, 139, 375, 458);
  rightColor.addCalibrationPoint(RED, 213, 505, 635);

  rightColor.addCalibrationPoint(RED, 194, 463, 564);
  leftColor.addCalibrationPoint(RED, 155, 368, 437);
  rightColor.addCalibrationPoint(RED, 188, 447, 548);
  leftColor.addCalibrationPoint(RED, 156, 365, 433);
  rightColor.addCalibrationPoint(RED, 199, 478, 588);
  leftColor.addCalibrationPoint(RED, 150, 369, 449);
  rightColor.addCalibrationPoint(RED, 204, 474, 574);
  leftColor.addCalibrationPoint(RED, 162, 393, 457);
  rightColor.addCalibrationPoint(RED, 196, 423, 518);
  leftColor.addCalibrationPoint(RED, 161, 372, 437);
  rightColor.addCalibrationPoint(RED, 225, 495, 597);
  leftColor.addCalibrationPoint(RED, 166, 414, 488);
  rightColor.addCalibrationPoint(RED, 238, 501, 605);
  leftColor.addCalibrationPoint(RED, 166, 382, 449);
  rightColor.addCalibrationPoint(RED, 218, 490, 595);
  leftColor.addCalibrationPoint(RED, 167, 408, 484);

  rightColor.addCalibrationPoint(RED, 204, 497, 609);
  leftColor.addCalibrationPoint(RED, 149, 353, 416);
  rightColor.addCalibrationPoint(RED, 206, 492, 605);
  leftColor.addCalibrationPoint(RED, 148, 344, 407);
  rightColor.addCalibrationPoint(RED, 202, 451, 550);
  leftColor.addCalibrationPoint(RED, 148, 384, 461);
  rightColor.addCalibrationPoint(RED, 209, 482, 610);
  leftColor.addCalibrationPoint(RED, 149, 346, 404);
  rightColor.addCalibrationPoint(RED, 204, 464, 575);
  leftColor.addCalibrationPoint(RED, 167, 375, 448);
  rightColor.addCalibrationPoint(RED, 214, 476, 576);
  leftColor.addCalibrationPoint(RED, 154, 367, 444);
  rightColor.addCalibrationPoint(RED, 203, 464, 559);
  leftColor.addCalibrationPoint(RED, 182, 424, 514);
  rightColor.addCalibrationPoint(RED, 203, 468, 575);
  leftColor.addCalibrationPoint(RED, 157, 324, 372);
  rightColor.addCalibrationPoint(RED, 198, 461, 564);
  leftColor.addCalibrationPoint(RED, 168, 372, 441);

  middleColor.addCalibrationPoint(YELLOW, 126, 214, 196);
  middleColor.addCalibrationPoint(YELLOW, 125, 214, 195);
  middleColor.addCalibrationPoint(YELLOW, 137, 224, 209);
  middleColor.addCalibrationPoint(YELLOW, 148, 233, 221);
  middleColor.addCalibrationPoint(YELLOW, 171, 244, 236);
  middleColor.addCalibrationPoint(YELLOW, 169, 248, 238);
  middleColor.addCalibrationPoint(YELLOW, 187, 250, 244);
  middleColor.addCalibrationPoint(YELLOW, 147, 231, 219);
  middleColor.addCalibrationPoint(YELLOW, 152, 238, 228);

  leftColor.addCalibrationPoint(YELLOW, 105, 267, 188);
  leftColor.addCalibrationPoint(YELLOW, 106, 269, 190);
  leftColor.addCalibrationPoint(YELLOW, 121, 292, 206);
  leftColor.addCalibrationPoint(YELLOW, 135, 292, 212);
  leftColor.addCalibrationPoint(YELLOW, 128, 281, 215);
  leftColor.addCalibrationPoint(YELLOW, 119, 271, 198);
  leftColor.addCalibrationPoint(YELLOW, 128, 286, 198);
  leftColor.addCalibrationPoint(YELLOW, 111, 275, 196);
  leftColor.addCalibrationPoint(YELLOW, 108, 285, 198);

  rightColor.addCalibrationPoint(YELLOW, 140, 350, 258);
  rightColor.addCalibrationPoint(YELLOW, 142, 364, 264);
  rightColor.addCalibrationPoint(YELLOW, 147, 346, 252);
  rightColor.addCalibrationPoint(YELLOW, 147, 322, 244);
  rightColor.addCalibrationPoint(YELLOW, 142, 347, 261);
  rightColor.addCalibrationPoint(YELLOW, 140, 323, 241);
  rightColor.addCalibrationPoint(YELLOW, 171, 367, 265);
  rightColor.addCalibrationPoint(YELLOW, 145, 344, 247);
  rightColor.addCalibrationPoint(YELLOW, 134, 331, 242);

  gripperColor.addCalibrationPoint(RED, 140, 341, 389);
  gripperColor.addCalibrationPoint(RED, 112, 322, 382);
  gripperColor.addCalibrationPoint(RED, 113, 350, 430);
  gripperColor.addCalibrationPoint(RED, 132, 365, 422);
  gripperColor.addCalibrationPoint(RED, 115, 359, 442);
  gripperColor.addCalibrationPoint(RED, 112, 347, 427);
  gripperColor.addCalibrationPoint(RED, 116, 347, 424);
  gripperColor.addCalibrationPoint(RED, 106, 347, 427);
  gripperColor.addCalibrationPoint(RED, 118, 328, 355);
  gripperColor.addCalibrationPoint(RED, 107, 337, 415);
  gripperColor.addCalibrationPoint(RED, 125, 366, 425);
  gripperColor.addCalibrationPoint(RED, 122, 338, 366);
  gripperColor.addCalibrationPoint(RED, 111, 354, 440);
  gripperColor.addCalibrationPoint(RED, 112, 363, 451);
  gripperColor.addCalibrationPoint(RED, 111, 335, 387);
  gripperColor.addCalibrationPoint(RED, 122, 372, 455);
  gripperColor.addCalibrationPoint(RED, 120, 359, 442);
  gripperColor.addCalibrationPoint(RED, 123, 265, 225);
  gripperColor.addCalibrationPoint(BLUE, 402, 130, 248);
  gripperColor.addCalibrationPoint(BLUE, 412, 131, 252);
  gripperColor.addCalibrationPoint(BLUE, 442, 143, 273);
  gripperColor.addCalibrationPoint(BLUE, 438, 138, 263);
  gripperColor.addCalibrationPoint(BLUE, 418, 142, 266);
  gripperColor.addCalibrationPoint(BLUE, 415, 127, 245);
  gripperColor.addCalibrationPoint(BLUE, 421, 134, 255);
  gripperColor.addCalibrationPoint(BLUE, 410, 129, 251);
  gripperColor.addCalibrationPoint(BLUE, 418, 125, 244);
  gripperColor.addCalibrationPoint(BLUE, 410, 124, 241);
  gripperColor.addCalibrationPoint(BLUE, 425, 130, 253);
  gripperColor.addCalibrationPoint(BLUE, 409, 127, 245);
  gripperColor.addCalibrationPoint(BLUE, 372, 148, 264);
  gripperColor.addCalibrationPoint(BLUE, 440, 141, 269);
  gripperColor.addCalibrationPoint(BLUE, 419, 133, 254);
  gripperColor.addCalibrationPoint(BLUE, 378, 147, 263);
  gripperColor.addCalibrationPoint(BLUE, 258, 138, 227);
  gripperColor.addCalibrationPoint(BLUE, 434, 128, 250);
  gripperColor.addCalibrationPoint(BLUE, 445, 134, 261);
  gripperColor.addCalibrationPoint(BLUE, 306, 128, 230);
  gripperColor.addCalibrationPoint(BLUE, 267, 120, 211);
  gripperColor.addCalibrationPoint(BLUE, 426, 125, 245);
  gripperColor.addCalibrationPoint(BLUE, 440, 134, 259);
  gripperColor.addCalibrationPoint(BLUE, 392, 123, 237);
  gripperColor.addCalibrationPoint(BLUE, 442, 124, 247);
  gripperColor.addCalibrationPoint(BLUE, 441, 124, 245);
  gripperColor.addCalibrationPoint(BLUE, 425, 125, 244);
  gripperColor.addCalibrationPoint(BLUE, 448, 139, 268);
  gripperColor.addCalibrationPoint(BLUE, 395, 125, 239);
  gripperColor.addCalibrationPoint(BLUE, 229, 192, 267);
  gripperColor.addCalibrationPoint(BLUE, 251, 175, 260);

  middleColor.addCalibrationPoint(BLACK, 396, 321, 433);
  middleColor.addCalibrationPoint(BLACK, 393, 318, 432);
  middleColor.addCalibrationPoint(BLACK, 395, 319, 432);
  middleColor.addCalibrationPoint(BLACK, 416, 341, 460);
  middleColor.addCalibrationPoint(BLACK, 415, 342, 462);
  middleColor.addCalibrationPoint(BLACK, 390, 319, 435);
  middleColor.addCalibrationPoint(BLACK, 314, 254, 340);
  middleColor.addCalibrationPoint(BLACK, 340, 275, 369);
  middleColor.addCalibrationPoint(BLACK, 343, 281, 386);
  middleColor.addCalibrationPoint(BLACK, 405, 331, 445);
  middleColor.addCalibrationPoint(BLACK, 383, 313, 417);
  middleColor.addCalibrationPoint(BLACK, 393, 322, 430);
  middleColor.addCalibrationPoint(BLACK, 402, 332, 445);
  middleColor.addCalibrationPoint(BLACK, 398, 329, 443);

  middleColor.addCalibrationPoint(BLACK, 354, 289, 392);
  middleColor.addCalibrationPoint(BLACK, 326, 264, 355);
  middleColor.addCalibrationPoint(BLACK, 372, 304, 411);
  middleColor.addCalibrationPoint(BLACK, 369, 304, 413);
  middleColor.addCalibrationPoint(BLACK, 371, 307, 413);
  middleColor.addCalibrationPoint(BLACK, 365, 304, 409);
  middleColor.addCalibrationPoint(BLACK, 351, 289, 392);
  middleColor.addCalibrationPoint(BLACK, 389, 321, 436);
  middleColor.addCalibrationPoint(BLACK, 369, 303, 409);
  middleColor.addCalibrationPoint(BLACK, 361, 297, 401);
  middleColor.addCalibrationPoint(BLACK, 353, 293, 404);
  middleColor.addCalibrationPoint(BLACK, 323, 265, 355);
  middleColor.addCalibrationPoint(BLACK, 325, 266, 358);
  middleColor.addCalibrationPoint(BLACK, 334, 274, 368);
  middleColor.addCalibrationPoint(BLACK, 340, 279, 372);
  middleColor.addCalibrationPoint(BLACK, 315, 258, 351);
  middleColor.addCalibrationPoint(BLACK, 370, 303, 409);
  middleColor.addCalibrationPoint(BLACK, 363, 296, 401);
  middleColor.addCalibrationPoint(BLACK, 357, 290, 393);
  middleColor.addCalibrationPoint(BLACK, 365, 298, 401);
  middleColor.addCalibrationPoint(BLACK, 333, 271, 366);
  middleColor.addCalibrationPoint(BLACK, 357, 290, 393);
  middleColor.addCalibrationPoint(BLACK, 334, 272, 367);
  middleColor.addCalibrationPoint(BLACK, 330, 271, 369);
  middleColor.addCalibrationPoint(BLACK, 382, 315, 427);
  middleColor.addCalibrationPoint(BLACK, 359, 294, 399);
  middleColor.addCalibrationPoint(BLACK, 374, 306, 416);
  middleColor.addCalibrationPoint(BLACK, 319, 267, 357);
  middleColor.addCalibrationPoint(BLACK, 345, 283, 382);
  middleColor.addCalibrationPoint(BLACK, 350, 290, 393);
  middleColor.addCalibrationPoint(BLACK, 351, 289, 389);
  middleColor.addCalibrationPoint(BLACK, 376, 315, 430);
  middleColor.addCalibrationPoint(BLACK, 322, 262, 351);
  middleColor.addCalibrationPoint(BLACK, 336, 276, 368);
  middleColor.addCalibrationPoint(BLACK, 328, 267, 361);
  middleColor.addCalibrationPoint(BLACK, 355, 299, 411);
  middleColor.addCalibrationPoint(BLACK, 326, 268, 361);
  middleColor.addCalibrationPoint(BLACK, 372, 307, 417);
  middleColor.addCalibrationPoint(BLACK, 343, 277, 373);
  middleColor.addCalibrationPoint(BLACK, 372, 305, 414);
  middleColor.addCalibrationPoint(BLACK, 336, 276, 371);
  middleColor.addCalibrationPoint(BLACK, 342, 280, 376);
  middleColor.addCalibrationPoint(BLACK, 385, 316, 426);
  middleColor.addCalibrationPoint(BLACK, 354, 292, 390);
  middleColor.addCalibrationPoint(BLACK, 340, 278, 375);
  middleColor.addCalibrationPoint(BLACK, 361, 298, 402);
  middleColor.addCalibrationPoint(BLACK, 360, 298, 404);
  middleColor.addCalibrationPoint(BLACK, 338, 277, 372);
  middleColor.addCalibrationPoint(BLACK, 353, 288, 384);
  middleColor.addCalibrationPoint(BLACK, 376, 304, 414);
  middleColor.addCalibrationPoint(BLACK, 378, 311, 423);
  middleColor.addCalibrationPoint(BLACK, 377, 307, 418);
  middleColor.addCalibrationPoint(BLACK, 343, 279, 379);
  middleColor.addCalibrationPoint(BLACK, 350, 284, 381);
  middleColor.addCalibrationPoint(BLACK, 334, 274, 372);
  middleColor.addCalibrationPoint(BLACK, 319, 258, 346);
  middleColor.addCalibrationPoint(BLACK, 377, 310, 419);
  middleColor.addCalibrationPoint(BLACK, 346, 284, 383);
  middleColor.addCalibrationPoint(BLACK, 344, 284, 384);
  middleColor.addCalibrationPoint(BLACK, 350, 288, 391);
  middleColor.addCalibrationPoint(BLACK, 340, 283, 378);
  middleColor.addCalibrationPoint(BLACK, 370, 303, 411);
  middleColor.addCalibrationPoint(BLACK, 377, 309, 421);
  middleColor.addCalibrationPoint(BLACK, 375, 306, 410);
  middleColor.addCalibrationPoint(BLACK, 360, 291, 382);
  middleColor.addCalibrationPoint(BLACK, 349, 277, 360);
  middleColor.addCalibrationPoint(BLACK, 367, 299, 403);
  middleColor.addCalibrationPoint(BLACK, 366, 295, 395);
  middleColor.addCalibrationPoint(BLACK, 392, 320, 434);
  middleColor.addCalibrationPoint(BLACK, 348, 285, 389);
  middleColor.addCalibrationPoint(BLACK, 328, 269, 360);
  middleColor.addCalibrationPoint(BLACK, 324, 257, 340);
  middleColor.addCalibrationPoint(BLACK, 372, 303, 412);
  middleColor.addCalibrationPoint(BLACK, 396, 325, 440);
  middleColor.addCalibrationPoint(BLACK, 396, 325, 442);
  middleColor.addCalibrationPoint(BLACK, 396, 324, 442);
  middleColor.addCalibrationPoint(BLACK, 395, 324, 441);

  middleColor.addCalibrationPoint(BLACK, 365, 293, 402);
  middleColor.addCalibrationPoint(BLACK, 370, 296, 405);
  middleColor.addCalibrationPoint(BLACK, 369, 296, 406);
  middleColor.addCalibrationPoint(BLACK, 367, 293, 403);
  middleColor.addCalibrationPoint(BLACK, 377, 302, 411);
  middleColor.addCalibrationPoint(BLACK, 329, 264, 360);
  middleColor.addCalibrationPoint(BLACK, 353, 281, 384);
  middleColor.addCalibrationPoint(BLACK, 371, 294, 407);
  middleColor.addCalibrationPoint(BLACK, 322, 257, 350);
  middleColor.addCalibrationPoint(BLACK, 313, 251, 347);
  middleColor.addCalibrationPoint(BLACK, 370, 298, 405);
  middleColor.addCalibrationPoint(BLACK, 392, 318, 434);
  middleColor.addCalibrationPoint(BLACK, 365, 294, 401);
  middleColor.addCalibrationPoint(BLACK, 301, 242, 327);
  middleColor.addCalibrationPoint(BLACK, 359, 290, 392);
  middleColor.addCalibrationPoint(BLACK, 332, 265, 362);
  middleColor.addCalibrationPoint(BLACK, 352, 285, 386);
  middleColor.addCalibrationPoint(BLACK, 344, 280, 378);
  middleColor.addCalibrationPoint(BLACK, 382, 311, 423);
  middleColor.addCalibrationPoint(BLACK, 384, 312, 428);
  middleColor.addCalibrationPoint(BLACK, 363, 294, 399);
  middleColor.addCalibrationPoint(BLACK, 365, 295, 404);
  middleColor.addCalibrationPoint(BLACK, 366, 300, 407);
  middleColor.addCalibrationPoint(BLACK, 363, 296, 399);
  middleColor.addCalibrationPoint(BLACK, 358, 292, 388);
  middleColor.addCalibrationPoint(BLACK, 369, 298, 403);
  middleColor.addCalibrationPoint(BLACK, 365, 298, 408);
  middleColor.addCalibrationPoint(BLACK, 330, 269, 367);
  middleColor.addCalibrationPoint(BLACK, 385, 313, 426);
  middleColor.addCalibrationPoint(BLACK, 382, 310, 423);
  middleColor.addCalibrationPoint(BLACK, 309, 248, 335);
  middleColor.addCalibrationPoint(BLACK, 354, 279, 374);
  middleColor.addCalibrationPoint(BLACK, 289, 236, 320);
  middleColor.addCalibrationPoint(BLACK, 373, 302, 412);
  middleColor.addCalibrationPoint(BLACK, 326, 264, 350);
  middleColor.addCalibrationPoint(BLACK, 374, 306, 422);
  middleColor.addCalibrationPoint(BLACK, 339, 276, 376);
  middleColor.addCalibrationPoint(BLACK, 328, 262, 353);
  middleColor.addCalibrationPoint(BLACK, 360, 292, 395);
  middleColor.addCalibrationPoint(BLACK, 348, 281, 378);
  middleColor.addCalibrationPoint(BLACK, 368, 290, 397);
  middleColor.addCalibrationPoint(BLACK, 337, 269, 353);
  middleColor.addCalibrationPoint(BLACK, 356, 287, 390);
  middleColor.addCalibrationPoint(BLACK, 379, 307, 421);
  middleColor.addCalibrationPoint(BLACK, 359, 292, 392);
  middleColor.addCalibrationPoint(BLACK, 365, 287, 389);
  middleColor.addCalibrationPoint(BLACK, 354, 289, 393);
  middleColor.addCalibrationPoint(BLACK, 345, 280, 380);
  middleColor.addCalibrationPoint(BLACK, 350, 282, 389);
  middleColor.addCalibrationPoint(BLACK, 373, 302, 414);
  middleColor.addCalibrationPoint(BLACK, 354, 285, 379);
  middleColor.addCalibrationPoint(BLACK, 349, 282, 387);
  middleColor.addCalibrationPoint(BLACK, 325, 267, 348);
  middleColor.addCalibrationPoint(BLACK, 322, 260, 343);
  middleColor.addCalibrationPoint(BLACK, 352, 288, 392);
  middleColor.addCalibrationPoint(BLACK, 366, 301, 406);
  middleColor.addCalibrationPoint(BLACK, 365, 297, 402);
  middleColor.addCalibrationPoint(BLACK, 364, 297, 402);

  middleColor.addCalibrationPoint(BLUE, 255, 109, 198);
  middleColor.addCalibrationPoint(BLUE, 255, 110, 198);
  middleColor.addCalibrationPoint(BLUE, 254, 109, 196);
  middleColor.addCalibrationPoint(BLUE, 254, 109, 195);
  middleColor.addCalibrationPoint(BLUE, 254, 108, 196);
  middleColor.addCalibrationPoint(BLUE, 248, 107, 193);
  middleColor.addCalibrationPoint(BLUE, 257, 113, 202);
  middleColor.addCalibrationPoint(BLUE, 260, 117, 206);
  middleColor.addCalibrationPoint(BLUE, 250, 113, 200);
  middleColor.addCalibrationPoint(BLUE, 256, 116, 206);
  middleColor.addCalibrationPoint(BLUE, 267, 120, 212);
  middleColor.addCalibrationPoint(BLUE, 268, 119, 212);
  middleColor.addCalibrationPoint(BLUE, 263, 118, 210);
  middleColor.addCalibrationPoint(BLUE, 270, 120, 214);
  middleColor.addCalibrationPoint(BLUE, 272, 122, 216);
  middleColor.addCalibrationPoint(BLUE, 262, 114, 205);
  middleColor.addCalibrationPoint(BLUE, 249, 114, 202);
  middleColor.addCalibrationPoint(BLUE, 264, 118, 208);
  middleColor.addCalibrationPoint(BLUE, 267, 119, 211);
  middleColor.addCalibrationPoint(BLUE, 271, 120, 213);
  middleColor.addCalibrationPoint(BLUE, 269, 120, 213);
  middleColor.addCalibrationPoint(BLUE, 279, 123, 219);
  middleColor.addCalibrationPoint(BLUE, 259, 117, 207);
  middleColor.addCalibrationPoint(BLUE, 264, 118, 210);
  middleColor.addCalibrationPoint(BLUE, 265, 118, 211);
  middleColor.addCalibrationPoint(BLUE, 268, 119, 213);
  middleColor.addCalibrationPoint(BLUE, 268, 123, 216);
  middleColor.addCalibrationPoint(BLUE, 279, 124, 220);
  middleColor.addCalibrationPoint(BLUE, 254, 115, 205);
  middleColor.addCalibrationPoint(BLUE, 270, 118, 211);
  middleColor.addCalibrationPoint(BLUE, 268, 117, 210);
  middleColor.addCalibrationPoint(BLUE, 267, 117, 210);
  middleColor.addCalibrationPoint(BLUE, 275, 116, 213);
  middleColor.addCalibrationPoint(BLUE, 260, 114, 205);
  middleColor.addCalibrationPoint(BLUE, 240, 110, 193);
  middleColor.addCalibrationPoint(BLUE, 251, 118, 205);
  middleColor.addCalibrationPoint(BLUE, 246, 106, 192);
  middleColor.addCalibrationPoint(BLUE, 243, 105, 189);
  middleColor.addCalibrationPoint(BLUE, 240, 105, 189);
  middleColor.addCalibrationPoint(BLUE, 241, 106, 188);
  middleColor.addCalibrationPoint(BLUE, 240, 105, 188);
  middleColor.addCalibrationPoint(BLUE, 244, 105, 189);
  middleColor.addCalibrationPoint(BLUE, 243, 105, 189);

  middleColor.addCalibrationPoint(BLACK, 357, 290, 392);
  middleColor.addCalibrationPoint(BLACK, 355, 286, 389);
  middleColor.addCalibrationPoint(BLACK, 350, 283, 384);
  middleColor.addCalibrationPoint(BLACK, 347, 280, 382);
  middleColor.addCalibrationPoint(BLACK, 363, 294, 398);
  middleColor.addCalibrationPoint(BLACK, 387, 313, 429);
  middleColor.addCalibrationPoint(BLACK, 377, 301, 407);
  middleColor.addCalibrationPoint(BLACK, 313, 250, 331);
  middleColor.addCalibrationPoint(BLACK, 335, 272, 370);
  middleColor.addCalibrationPoint(BLACK, 347, 283, 386);
  middleColor.addCalibrationPoint(BLACK, 372, 301, 412);
  middleColor.addCalibrationPoint(BLACK, 393, 321, 432);
  middleColor.addCalibrationPoint(BLACK, 400, 325, 451);
  middleColor.addCalibrationPoint(BLACK, 388, 314, 432);
  middleColor.addCalibrationPoint(BLACK, 353, 286, 390);
  middleColor.addCalibrationPoint(BLACK, 348, 280, 381);
  middleColor.addCalibrationPoint(BLACK, 348, 280, 381);
  middleColor.addCalibrationPoint(BLACK, 345, 280, 380);
  middleColor.addCalibrationPoint(BLACK, 330, 267, 360);
  middleColor.addCalibrationPoint(BLACK, 356, 290, 394);
  middleColor.addCalibrationPoint(BLACK, 361, 293, 398);
  middleColor.addCalibrationPoint(BLACK, 377, 309, 413);
  middleColor.addCalibrationPoint(BLACK, 374, 308, 417);
  middleColor.addCalibrationPoint(BLACK, 358, 293, 392);
  middleColor.addCalibrationPoint(BLACK, 393, 322, 440);
  middleColor.addCalibrationPoint(BLACK, 340, 276, 374);
  middleColor.addCalibrationPoint(BLACK, 346, 277, 374);
  middleColor.addCalibrationPoint(BLACK, 380, 308, 421);
  middleColor.addCalibrationPoint(BLACK, 342, 276, 368);
  middleColor.addCalibrationPoint(BLACK, 397, 323, 438);
  middleColor.addCalibrationPoint(BLACK, 398, 322, 441);
  middleColor.addCalibrationPoint(BLACK, 334, 274, 370);
  middleColor.addCalibrationPoint(BLACK, 355, 284, 387);
  middleColor.addCalibrationPoint(BLACK, 373, 300, 408);
  middleColor.addCalibrationPoint(BLACK, 339, 280, 375);
  middleColor.addCalibrationPoint(BLACK, 303, 247, 332);
  middleColor.addCalibrationPoint(BLACK, 382, 310, 423);
  middleColor.addCalibrationPoint(BLACK, 354, 292, 397);
  middleColor.addCalibrationPoint(BLACK, 317, 260, 356);
  middleColor.addCalibrationPoint(BLACK, 405, 331, 451);
  middleColor.addCalibrationPoint(BLACK, 380, 314, 426);
  middleColor.addCalibrationPoint(BLACK, 309, 252, 341);
  middleColor.addCalibrationPoint(BLACK, 338, 271, 368);
  middleColor.addCalibrationPoint(BLACK, 360, 292, 396);
  middleColor.addCalibrationPoint(BLACK, 323, 259, 351);
  middleColor.addCalibrationPoint(BLACK, 321, 256, 350);
  middleColor.addCalibrationPoint(BLACK, 369, 303, 416);
  middleColor.addCalibrationPoint(BLACK, 329, 262, 352);
  middleColor.addCalibrationPoint(BLACK, 379, 310, 424);
  middleColor.addCalibrationPoint(BLACK, 332, 272, 367);
  middleColor.addCalibrationPoint(BLACK, 368, 301, 407);
  middleColor.addCalibrationPoint(BLACK, 337, 271, 371);
  middleColor.addCalibrationPoint(BLACK, 339, 272, 371);

  rightColor.addCalibrationPoint(WHITE, 118, 114, 130);
  rightColor.addCalibrationPoint(WHITE, 112, 106, 122);
  rightColor.addCalibrationPoint(WHITE, 114, 107, 123);
  rightColor.addCalibrationPoint(WHITE, 131, 124, 142);
  rightColor.addCalibrationPoint(WHITE, 152, 134, 158);
  middleColor.addCalibrationPoint(WHITE, 101, 80, 105);
  middleColor.addCalibrationPoint(WHITE, 97, 78, 102);
  middleColor.addCalibrationPoint(WHITE, 104, 81, 108);
  middleColor.addCalibrationPoint(WHITE, 105, 84, 110);
  middleColor.addCalibrationPoint(WHITE, 96, 78, 102);
  leftColor.addCalibrationPoint(WHITE, 118, 119, 129);
  leftColor.addCalibrationPoint(WHITE, 117, 118, 128);
  leftColor.addCalibrationPoint(WHITE, 118, 120, 130);
  leftColor.addCalibrationPoint(WHITE, 126, 118, 131);
  leftColor.addCalibrationPoint(WHITE, 131, 117, 131);
}
#endif