/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 Quasics Robotics and Matthew J. Healy                   */
/* All Rights Reserved.                                                       */
/*                                                                            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the BSD license file in the root directory of the   */
/* project repository.                                                        */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cmath>

/**
 * Utility class/functor to convert ticks (reported by an encoder) to inches or
 * other "human units".
 */
class EncoderTicksToUnitsConverter {
 public:
  /**
   * Constructor
   *
   * @param ticksPerRevolution ticks reported by an encoder for a full
   *                           revolution of the motor
   * @param gearRatio ratio of motor revolutions to wheel revolutions
   * @param wheelDiameter diameter of the wheel, in output units
   *
   * @see #GetEncoderPosition
   */
  constexpr EncoderTicksToUnitsConverter(double ticksPerMotorRevolution,
                                         double gearRatio, double wheelDiameter)
      : ticksPerMotorRevolution(ticksPerMotorRevolution),
        gearRatio(gearRatio),
        wheelDiameter(wheelDiameter) {
  }

  /**
   * Converts the specified number of ticks on the encoder to "human units".
   */
  double operator()(double ticks) const {
    // Formula for encoder value to inches: (encoder output in ticks)/(ticks
    // per revolution)/(gearRatio)*(wheelDiameter * Pi)
    return ((ticks / ticksPerMotorRevolution) / gearRatio) *
           (wheelDiameter * PI);
  }

 private:
  /// Constant for Pi.  (Not included in math library until C++20.)
  static constexpr double PI = 4.0 * std::atan(1);
  const double ticksPerMotorRevolution;
  const double gearRatio;
  const double wheelDiameter;
};
