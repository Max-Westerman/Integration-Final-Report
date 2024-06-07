#ifndef ULTRASONIC_H
#define ULTRASONIC_H
// HC-SR04 Ultrasonic Sensor
// =====================

#include <Arduino.h>

class UltraSonic {
  public: 
  int echoPin;
  int trigPin;
  const char* label;
  int soundDelay;
  float distance;

  void initialize() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  float getDuration() {

    delay(10); // If pulses are too fast it can cause a 1000ms delay

    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(soundDelay);
    digitalWrite(trigPin, LOW);
    float sound_duration = pulseIn(echoPin, HIGH);

    return sound_duration;
  }

  float getDistance() {
    distance = getDuration() * 0.034 / 2;
    return distance;
  }

};

#endif