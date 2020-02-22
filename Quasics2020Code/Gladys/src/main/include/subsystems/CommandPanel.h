/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/util/color.h>
#include <frc2/command/SubsystemBase.h>

#include <string>

#include "Constants.h"
#include "rev/ColorMatch.h"
#include "rev/ColorSensorV3.h"

// TODO(RJ): Document this class.
class CommandPanel : public frc2::SubsystemBase {
 public:
  enum Color { NO_DATA = -1, UNKNOWN = 0, BLUE = 1, RED = 2, GREEN = 3, YELLOW = 4 };

  CommandPanel();
  //~CommandPanel();
  void Periodic();
  // Motor control
  void TurnWheelMotorOff();

  void TurnWheelMotorOn(bool);
  // changes the color recieved to an int
  Color getCurrentColor();
  static std::string getColorName(Color c);

 private:
  typedef ctre::phoenix::motorcontrol::can::WPI_VictorSPX WPI_VictorSPX;
  WPI_VictorSPX motor;

  rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};
  rev::ColorMatch m_colorMatcher;
};
