/**
 * @file ColorSensor.h
 * @brief Defines the ColorSensor class for interfacing with the TCS230/TCS3200 color sensors.
 * 
 * This file contains the definition of the ColorSensor class, which is used to manage
 * and interpret color data from the TCS230/TCS3200 RGB light color recognition sensors.
 * 
 * Created by: [Your Name]
 * Date: [Date]
 * 
 * Modifications:
 * Date: [Date] [Description of modification]
 */

#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <math.h>
#include <Arduino.h>
#include <deque>
#include <map>
#include <vector> 

enum Color {
  RED,
  GREEN,
  BLUE,
  YELLOW,
  BLACK,
  WHITE,
  UNKNOWN,
};

typedef struct {
  Color color;
  int values[3];
} CalibrationPoint;

class ColorSensor {
  public:
    int output_frequency_0_pin, output_frequency_1_pin;
    int color_selector_2_pin, color_selector_3_pin;
    int out_pin;

    Color color;
    int red, green, blue;
    int frequency;

    const char* label;
    int moving_average_window = 3;
    float distance_trigger = 10000; // Won't count the color unless it's closer than this

    const char* calibration_color;
    const char* calibration_name;

    std::vector<CalibrationPoint> calibration;
    std::deque<Color> color_history;

    /**
     * @brief Calculates the Euclidean distance between two colors.
     * 
     * @param color1 The first color's RGB values.
     * @param color2 The second color's RGB values.
     * @return The Euclidean distance between the two colors.
     */
    float calculateEuclideanDistance(const int color1[3], const int color2[3]) {
      return sqrt(pow(color2[0] - color1[0], 2) + pow(color2[1] - color1[1], 2) + pow(color2[2] - color1[2], 2));
    }

    /**
     * @brief Adds a calibration point for a specific color.
     * 
     * @param color The color to add.
     * @param red The red component of the color.
     * @param green The green component of the color.
     * @param blue The blue component of the color.
     */
    void addCalibrationPoint(Color color, int red, int green, int blue) {
      CalibrationPoint newPoint = {color, {red, green, blue}};
      calibration.push_back(newPoint);
    }

    /**
     * @brief Sets the frequency scaling for the sensor.
     */
    void setFrequencyScaling() {
      switch (frequency) {
        case 0:
          digitalWrite(output_frequency_0_pin, LOW);
          digitalWrite(output_frequency_1_pin, LOW);
          break;
        case 2:
          digitalWrite(output_frequency_0_pin, LOW);
          digitalWrite(output_frequency_1_pin, HIGH);
          break;
        case 20:
          digitalWrite(output_frequency_0_pin, HIGH);
          digitalWrite(output_frequency_1_pin, LOW);
          break;
        case 100:
          digitalWrite(output_frequency_0_pin, HIGH);
          digitalWrite(output_frequency_1_pin, HIGH);
          break;
      }
    }

    /**
     * @brief Reads the color frequency from the sensor.
     * 
     * @param s2_value The value to set for the S2 pin.
     * @param s3_value The value to set for the S3 pin.
     * @return The color frequency.
     */
    int readColorFrequency(int s2_value, int s3_value) {
      digitalWrite(color_selector_2_pin, s2_value);
      digitalWrite(color_selector_3_pin, s3_value);
      return pulseIn(out_pin, LOW);
    }

    /**
     * @brief Reads the RGB values from the sensor.
     */
    void readRGB() {
      red = readColorFrequency(LOW, LOW);
      green = readColorFrequency(LOW, HIGH);
      blue = readColorFrequency(HIGH, HIGH);
    }

    /**
     * @brief Determines the current color based on the RGB readings.
     * 
     * @return The detected color.
     */
    Color getColor() {
      readRGB();
      int sensor_rgb_readings[3] = {red, green, blue};
      float minDistance = 100000;
      color = UNKNOWN;
      Color return_color = UNKNOWN;

      for (const auto& cal : calibration) {
        float distance = calculateEuclideanDistance(sensor_rgb_readings, cal.values);
        if (distance < minDistance) {
          minDistance = distance;
          color = cal.color;
        }
      }

      if (minDistance < distance_trigger){
        color_history.push_back(color);
        if (color_history.size() > static_cast<unsigned int>(moving_average_window)) {
          color_history.pop_front();
        }
        return_color = getMovingAverageColor();
      }
      return return_color;
    }

    /**
     * @brief Calculates the moving average color from the color history.
     * 
     * @return The most frequent color in the history.
     */
    Color getMovingAverageColor() {
      if (color_history.size() < static_cast<unsigned int>(moving_average_window)) {
        return color_history.back();
      }

      std::map<Color, int> color_count;
      for (const auto& col : color_history) {
        color_count[col]++;
      }

      Color most_frequent_color = color_history.back();
      int max_count = 0;
      for (const auto& entry : color_count) {
        if (entry.second > max_count) {
          max_count = entry.second;
          most_frequent_color = entry.first;
        }
      }

      if (max_count >= moving_average_window / 2 + 1) {
        return most_frequent_color;
      } else {
        return color_history[color_history.size() - 2];
      }
    }

    /**
     * @brief Clears the color history.
     */
    void clearColorHistory() {
      color_history.clear();
    }

    /**
     * @brief Prints out calibration data to the serial monitor.
     */
    void calibration_printout(){
      getColor();
      Serial.print(calibration_name);
      Serial.print(".addCalibrationPoint(");
      Serial.print(calibration_color);
      Serial.print(", ");
      Serial.print(red);
      Serial.print(", ");
      Serial.print(green);
      Serial.print(", ");
      Serial.print(blue);
      Serial.println(");");
    }

    /**
     * @brief Initializes the sensor pins and settings.
     */
    void initialize() {
      red = 0;
      green = 0;
      blue = 0;
      color = UNKNOWN;
      pinMode(output_frequency_0_pin, OUTPUT);
      pinMode(output_frequency_1_pin, OUTPUT);
      pinMode(color_selector_2_pin, OUTPUT);
      pinMode(color_selector_3_pin, OUTPUT);
      pinMode(out_pin, INPUT);
      setFrequencyScaling();
    }
};

#endif // COLOR_SENSOR_H
