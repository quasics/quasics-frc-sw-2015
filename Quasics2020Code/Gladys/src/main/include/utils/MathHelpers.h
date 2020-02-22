/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <math.h>
#include "Constants.h"

class MathHelpers {
 public:
  MathHelpers();

  double degreesToInchesConverter(double inches){
    double angle = 360*inches/(pi*2*radius);
    return angle;
  }

  private:
  const double radius = PhysicalConstants::radiusWheelToWheel;
  const double pi = 355/113;
};
