/**
 * @file IRSensorArray.h
 * @brief Defines the IRSensorArray class for handling an array of IR sensors.
 * 
 * This file contains the definition of the IRSensorArray class, which is used to manage
 * and process data from an array of infrared sensors, similar to the QTR-8RC Reflectance Sensor Array.
 * 
 * Created by: [Your Name]
 * Date: [Date]
 * 
 * Modifications:
 * Date: [Date] [Description of modification]
 */

#ifndef IR_SENSOR_ARRAY_H
#define IR_SENSOR_ARRAY_H

#include <Arduino.h>
#include "ColorSensor.h"

class IRSensorArray {
  private:
    struct CalibrationValues {
      int* onValues;
      int* offValues;
      int* thresholds;
    };
    const int windowSize = 3;  ///< Moving average window size
    int** sensorHistory;       ///< Buffer for sensor readings
    int* historyIndex;         ///< Current index in the buffer

  public:
    int numSensors;
    int* sensorPins;
    int* sensorValues;
    int* sensorTriggers;
    bool debug = false;
    unsigned long initial_warmup_duration = 600;
    CalibrationValues calValues[4];  ///< For RED, GREEN, BLUE, YELLOW
    Color currentColor = BLUE;       ///< Current color setting
    float error;

    /**
     * @brief Constructor for IRSensorArray.
     */
    IRSensorArray() {}

    /**
     * @brief Destructor for IRSensorArray.
     * 
     * Frees dynamically allocated memory.
     */
    ~IRSensorArray() {
      delete[] sensorValues;
      delete[] sensorTriggers;
      for (int i = 0; i < 4; i++) {
        delete[] calValues[i].onValues;
        delete[] calValues[i].offValues;
        delete[] calValues[i].thresholds;
      }
      for (int i = 0; i < numSensors; i++) {
        delete[] sensorHistory[i];
      }
      delete[] sensorHistory;
      delete[] historyIndex;
    }

    /**
     * @brief Initializes the IR sensor array.
     */
    void initialize() {
      sensorValues = new int[numSensors];
      sensorTriggers = new int[numSensors];
      for (int i = 0; i < 4; i++) {
        calValues[i].onValues = new int[numSensors];
        calValues[i].offValues = new int[numSensors];
        calValues[i].thresholds = new int[numSensors];
      }
      sensorHistory = new int*[numSensors];
      historyIndex = new int[numSensors];

      for (int i = 0; i < numSensors; i++) {
        sensorHistory[i] = new int[windowSize]();
        historyIndex[i] = 0;
      }

      for (int i = 0; i < numSensors; i++) {
        pinMode(sensorPins[i], INPUT);
      }

      unsigned long startTime = millis();
      while (millis() - startTime < initial_warmup_duration) {
        getError();
      }
    }

    /**
     * @brief Reads the sensor values and updates the moving average.
     */
    void readSensors() {
      for (int i = 0; i < numSensors; i++) {
        sensorHistory[i][historyIndex[i]] = analogRead(sensorPins[i]);
        historyIndex[i] = (historyIndex[i] + 1) % windowSize;

        int sum = 0;
        for (int j = 0; j < windowSize; j++) {
          sum += sensorHistory[i][j];
        }
        sensorValues[i] = sum / windowSize;
        sensorTriggers[i] = sensorValues[i] > calValues[currentColor].thresholds[i];
      }
    }

    /**
     * @brief Sets the calibration values for a specified color.
     * 
     * @param color The color to set calibration values for.
     * @param onValues The on values for the sensors.
     * @param offValues The off values for the sensors.
     */
    void setCalibrationValues(Color color, int* onValues, int* offValues) {
      for (int i = 0; i < numSensors; i++) {
        calValues[color].onValues[i] = onValues[i];
        calValues[color].offValues[i] = offValues[i];
        calValues[color].thresholds[i] = (calValues[color].onValues[i] + calValues[color].offValues[i]) / 2;
      }
    }

    /**
     * @brief Returns the current sensor values.
     * 
     * @return Pointer to the array of sensor values.
     */
    int* getSensorValues() {
      return sensorValues;
    }

    /**
     * @brief Checks if a specific sensor is triggered.
     * 
     * @param index The index of the sensor to check.
     * @return True if the sensor is triggered, false otherwise.
     */
    bool isSensorTriggered(int index) {
      if (calValues[currentColor].onValues[index] > calValues[currentColor].offValues[index]) {
        return sensorValues[index] > calValues[currentColor].thresholds[index];
      } else {
        return sensorValues[index] < calValues[currentColor].thresholds[index];
      }
    }

    /**
     * @brief Calculates the error based on sensor readings.
     * 
     * @return The calculated error.
     */
    float getError() {
      readSensors();
      float sumLeftWeight = 0;
      float sumRightWeight = 0;
      int countLeftWeight = 0;
      int countRightWeight = 0;
      int leftPoint;
      int rightPoint;

      if (numSensors % 2 == 0) {
        leftPoint = numSensors / 2 - 1;
        rightPoint = numSensors / 2;
      } else {
        leftPoint = numSensors / 2;
        rightPoint = leftPoint + 1;
      }

      for (int i = 0; i < numSensors; i++) {
        float weight = (-1 + 2 * ((float)i / (numSensors - 1))) * isSensorTriggered(i);

        if (debug) {
          Serial.print("Sensor ");
          Serial.print(i);
          Serial.print(": Triggered = ");
          Serial.print(isSensorTriggered(i));
          Serial.print(", Weight = ");
          Serial.println(weight);
        }

        if (i <= leftPoint && weight != 0) {
          sumLeftWeight += weight;
          countLeftWeight++;
        } else if (i >= rightPoint && weight != 0) {
          sumRightWeight += weight;
          countRightWeight++;
        }
      }

      float avgLeftWeight = (countLeftWeight > 0) ? (sumLeftWeight / countLeftWeight) : 0;
      float avgRightWeight = (countRightWeight > 0) ? (sumRightWeight / countRightWeight) : 0;

      error = avgLeftWeight + avgRightWeight;

      if (debug) {
        Serial.print("sumLeftWeight: ");
        Serial.println(sumLeftWeight);
        Serial.print("sumRightWeight: ");
        Serial.println(sumRightWeight);
        Serial.print("countLeftWeight: ");
        Serial.println(countLeftWeight);
        Serial.print("countRightWeight: ");
        Serial.println(countRightWeight);
        Serial.print("avgLeftWeight: ");
        Serial.println(avgLeftWeight);
        Serial.print("avgRightWeight: ");
        Serial.println(avgRightWeight);
        Serial.print("Total Error: ");
        Serial.println(error);
      }
      return error;
    }

    /**
     * @brief Prints the current error and sensor trigger states.
     */
    void printout() {
      Serial.print("Error ");
      Serial.print(getError());
      Serial.print(" | ");
      for (int i = 0; i < numSensors; i++) {
        Serial.print("S");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(isSensorTriggered(i));
        Serial.print(" | ");
      }
      Serial.println();
    }

    /**
     * @brief Prints the current calibration values.
     */
    void calibrate_printout() {
      readSensors();
      Serial.print("{");
      for (int i = 0; i < numSensors; i++) {
        Serial.print(sensorValues[i]);
        if (i < numSensors - 1) {
          Serial.print(", ");
        }
      }
      Serial.println("}");
    }

    /**
     * @brief Sets the current color for calibration.
     * 
     * @param color The color to set.
     */
    void setColor(Color color) {
      currentColor = color;
    }
};

#endif // IR_SENSOR_ARRAY_H
