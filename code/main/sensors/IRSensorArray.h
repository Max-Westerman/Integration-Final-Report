#ifndef IR_SENSOR_ARRAY_H
#define IR_SENSOR_ARRAY_H
// Similar to QTR-8RC Reflectance Sensor Array
// =====================

#include <Arduino.h>
#include "ColorSensor.h"

class IRSensorArray {
  private:
    struct CalibrationValues {
      int* onValues;
      int* offValues;
      int* thresholds;
    };
    const int windowSize = 3;         // Moving average window size
    int** sensorHistory;              // Buffer for sensor readings
    int* historyIndex;                // Current index in the buffer

  public:
    int numSensors;
    int* sensorPins;
    int* sensorValues;
    int* sensorTriggers;
    bool debug = false;
    unsigned long initial_warmup_duration = 600;
    CalibrationValues calValues[4];   // For RED, GREEN, BLUE, YELLOW
    Color currentColor = BLUE;        // Current color setting
    float error;

    IRSensorArray() {}

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

      // Takes a second for the sensor to warm up
      unsigned long startTime = millis();
      while (millis() - startTime < initial_warmup_duration) {
        getError();
      }
    }

    void readSensors() {
      for (int i = 0; i < numSensors; i++) {
        // Update the history buffer with the latest reading
        sensorHistory[i][historyIndex[i]] = analogRead(sensorPins[i]);
        historyIndex[i] = (historyIndex[i] + 1) % windowSize;

        // Calculate the moving average
        int sum = 0;
        for (int j = 0; j < windowSize; j++) {
          sum += sensorHistory[i][j];
        }
        sensorValues[i] = sum / windowSize;

        // Determine if the sensor is triggered
        sensorTriggers[i] = sensorValues[i] > calValues[currentColor].thresholds[i];
      }
    }

    void setCalibrationValues(Color color, int* onValues, int* offValues) {
      for (int i = 0; i < numSensors; i++) {
        calValues[color].onValues[i] = onValues[i];
        calValues[color].offValues[i] = offValues[i];
        calValues[color].thresholds[i] = (calValues[color].onValues[i] + calValues[color].offValues[i]) / 2;
      }
    }

    int* getSensorValues() {
      return sensorValues;
    }

    bool isSensorTriggered(int index) {
      // Compare sensor value against the threshold considering on and off values for the current color
      if (calValues[currentColor].onValues[index] > calValues[currentColor].offValues[index]) {
        return sensorValues[index] > calValues[currentColor].thresholds[index];
      } else {
        return sensorValues[index] < calValues[currentColor].thresholds[index];
      }
    }

    float getError() {
      readSensors();
      float sumLeftWeight = 0;
      float sumRightWeight = 0;
      int countLeftWeight = 0;
      int countRightWeight = 0;
      int leftPoint;
      int rightPoint;

      // Determine the split points
      if (numSensors % 2 == 0) {
        leftPoint = numSensors / 2 - 1;
        rightPoint = numSensors / 2;
      } else {
        leftPoint = numSensors / 2;
        rightPoint = leftPoint + 1;
      }

      for (int i = 0; i < numSensors; i++) {
        // Weights from -1 to 1, leftmost sensor is -1 and rightmost is 1
        float weight = (-1 + 2 * ((float)i / (numSensors - 1))) * isSensorTriggered(i);

        if (debug) {
          // Debug prints
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
        // Debug prints
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

    void printout() {
      Serial.print("Error ");
      Serial.print(getError());
      Serial.print(" | ");
      for (int i = 0; i < numSensors; i++) {
        Serial.print("S");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(isSensorTriggered(i));
        // Serial.print(sensorValues[i]);
        Serial.print(" | ");
      }
      Serial.println();
    }

    void calibrate_printout() {
      readSensors(); // Ensure sensor values are updated before printing
      Serial.print("{");
      for (int i = 0; i < numSensors; i++) {
        Serial.print(sensorValues[i]);
        if (i < numSensors - 1) {
          Serial.print(", ");
        }
      }
      Serial.println("}");
    }

    void setColor(Color color) {
      currentColor = color;
    }
};

#endif