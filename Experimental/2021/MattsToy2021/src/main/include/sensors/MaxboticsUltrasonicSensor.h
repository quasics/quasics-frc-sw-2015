#pragma once

#include <frc/AnalogInput.h>
#include <frc/shuffleboard/Shuffleboard.h>

/**
 * Wrapper for access to a Maxbotics MB1043 ultrasonic sensor.
 *
 * Note: maximum sensed range is 5 Meters, and the minimum range is 30
 * centimeters.
 */
class MaxboticsUltrasonicSensor {
  static constexpr double kRawReadingToCentimeterConstant = 0.125;
  static constexpr double kRawReadingToInchesConstant = 0.049212598;

 public:
  MaxboticsUltrasonicSensor(int analogInput, bool addToShuffleboard = true)
      : m_sensor(analogInput) {
    if (addToShuffleboard) {
      auto &tab = frc::Shuffleboard::GetTab("Sensors");
      tab.Add("Ultrasonic", m_sensor);
    }
  }

  /**
   * Returns the raw reading from the sensor, as a value in the range [0..4095],
   * where 0 represents 0V applied, and 4095 represents 5V applied. The distance
   * detected from any sensed object can be computed as a linear result from
   * that.
   *
   * @see #GetDistance
   * @see https://www.maxbotix.com/firstrobotics
   */
  double GetRawValue() {
    return m_sensor.GetValue();
  }

  /**
   * Measurements units supported for the sensor's results.
   * @see #GetDistance
   */
  enum Unit { Inches, Centimeters };

  /**
   * Returns the distance measured from any object in the sensor's range, or -1
   * on an error.
   */
  double GetDistance(Unit unit) {
    switch (unit) {
      case Centimeters:
        return GetRawValue() * kRawReadingToCentimeterConstant;
      case Inches:
        return GetRawValue() * kRawReadingToInchesConstant;
    }

    // Error: somehow, we were given an unexpected mode.
    return -1;
  }

 private:
  frc::AnalogInput m_sensor;
};