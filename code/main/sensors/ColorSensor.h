#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H
// TCS230 TCS3200 RGB Light Color Recognition Sensor
// =====================

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
  float distance_trigger = 10000; //Won't count the color unless it's closer than this
  
  const char* calibration_color;
  const char* calibration_name;

  std::vector<CalibrationPoint> calibration;
  std::deque<Color> color_history;

  float calculateEuclideanDistance(const int color1[3], const int color2[3]) {
    return sqrt(pow(color2[0] - color1[0], 2) + pow(color2[1] - color1[1], 2) + pow(color2[2] - color1[2], 2));
  }

  void addCalibrationPoint(Color color, int red, int green, int blue) {
    CalibrationPoint newPoint = {color, {red, green, blue}};
    calibration.push_back(newPoint);
  }

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

  int readColorFrequency(int s2_value, int s3_value) {
    digitalWrite(color_selector_2_pin, s2_value);
    digitalWrite(color_selector_3_pin, s3_value);
    return pulseIn(out_pin, LOW);
  }

  void readRGB() {
    red = readColorFrequency(LOW, LOW);
    green = readColorFrequency(LOW, HIGH);
    blue = readColorFrequency(HIGH, HIGH);
  }

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
      // Add the current color to the history buffer
      color_history.push_back(color);
      if (color_history.size() > static_cast<unsigned int>(moving_average_window)) {
        color_history.pop_front();
      }
      return_color = getMovingAverageColor();
    }
    return return_color;
  }

  Color getMovingAverageColor() {
    if (color_history.size() < static_cast<unsigned int>(moving_average_window)) {
      return color_history.back(); // Not enough data, return the latest color
    }

    std::map<Color, int> color_count;
    for (const auto& col : color_history) {
      color_count[col]++;
    }

    // Find the color with the maximum count
    Color most_frequent_color = color_history.back(); // Default to the latest color
    int max_count = 0;
    for (const auto& entry : color_count) {
      if (entry.second > max_count) {
        max_count = entry.second;
        most_frequent_color = entry.first;
      }
    }

    // Ensure the most frequent color appears more than once in the window
    if (max_count >= moving_average_window / 2 + 1) {
      return most_frequent_color;
    } else {
      return color_history[color_history.size() - 2]; // Return the previous color
    }
  }

  void clearColorHistory() {
    color_history.clear();
  }

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
