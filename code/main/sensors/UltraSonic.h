/**
 * @file UltraSonic.h
 * @brief Defines the UltraSonic class for interfacing with the HC-SR04 ultrasonic sensor.
 *
 * This file contains the definition of the UltraSonic class, which is used to control
 * and retrieve distance measurements from an HC-SR04 ultrasonic sensor. This class handles
 * the initialization of sensor pins and the calculation of distance based on the sensor's
 * echo times.
 *
 * Created by: Max Westerman
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

/**
 * @class UltraSonic
 * @brief Class to interact with the HC-SR04 ultrasonic sensor.
 *
 * The UltraSonic class encapsulates methods for initializing the HC-SR04 ultrasonic sensor,
 * sending ultrasonic pulses, and calculating the distance to the nearest object based on
 * the echo received.
 */
class UltraSonic {
  public:
    int echoPin; ///< Pin connected to the echo pin of the sensor.
    int trigPin; ///< Pin connected to the trigger pin of the sensor.
    const char* label; ///< Optional label for identifying the sensor.
    int soundDelay; ///< Delay in microseconds between trigger and echo, affecting pulse frequency.
    float distance; ///< Last calculated distance from the sensor in centimeters.

    /**
     * Initializes the sensor pins.
     */
    void initialize() {
        pinMode(trigPin, OUTPUT); // Set trigger pin as output
        pinMode(echoPin, INPUT);  // Set echo pin as input
    }

    /**
     * Sends an ultrasonic pulse and measures the duration of the received echo.
     * @return Duration of the echo pulse in microseconds.
     */
    float getDuration() {
        delay(10); // Ensures there's no carryover from previous pulses

        digitalWrite(trigPin, LOW); // Clear the trigger pin
        delayMicroseconds(5); // Short delay before sending the pulse

        digitalWrite(trigPin, HIGH); // Send a high pulse
        delayMicroseconds(soundDelay); // Wait for soundDelay microseconds
        digitalWrite(trigPin, LOW); // End the pulse

        float sound_duration = pulseIn(echoPin, HIGH); // Measure the length of the incoming pulse

        return sound_duration;
    }

    /**
     * Calculates and returns the distance to the nearest object.
     * @return Distance to the nearest object in centimeters.
     */
    float getDistance() {
        distance = getDuration() * 0.034 / 2; // Convert time to distance
        return distance;
    }

};

#endif // ULTRASONIC_H
