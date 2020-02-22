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
  /**
  * Initializes CommandPanel class, sets subsystem, and adds color matches
  **/
  CommandPanel();
  //~CommandPanel();
    /**
  * Currently does nothing, will remove if nothing needs to be done.
  **/
  void Periodic();
  // Motor control
    /**
  * Turns the CommandPanel wheel off, does not return anything, and accepts no parameters.
  **/
  void TurnWheelMotorOff();
    /**
  * Turns the CommandPanel wheel on.
  * @param forward specifies the direction the motor is going 
  **/
  void TurnWheelMotorOn(bool);
  /**
  * Returns a Color enum, which is used to determine the current color
  * @return returns a Color enum value that corresponds to the color recieved. 
  **/
  Color getCurrentColor();
   /**
  * To easily convert a Color enum to a string.
  * @param color A color enum
  * @return returns a String that corresponds to the Color inputted. 
  **/
  static std::string getColorName(Color c);

 private:
  //the motor that spins the wheel
  typedef ctre::phoenix::motorcontrol::can::WPI_VictorSPX WPI_VictorSPX;
  WPI_VictorSPX motor;
  // Color objects
  // Color sensor gets the data from the sensor from the i^2c port
  rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};
  // Color matcher matches the data from the color sensor to the correct color.
  rev::ColorMatch m_colorMatcher;
};
