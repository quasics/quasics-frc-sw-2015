/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cmath>
#include "Constants.h"

class MathHelpers {
 public:
  static double inchesToDegreesConverter(double inches){
    double angle = 360*inches/(pi*2*radius);
    return angle;
  }

 private:
  static constexpr double radius = PhysicalConstants::radiusWheelToWheel;
  static constexpr double pi = 4.0 * std::atan(1);
};
