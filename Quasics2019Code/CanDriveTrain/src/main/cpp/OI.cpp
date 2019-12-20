/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include "ControllerDefinitions.h"

#include <frc/WPILib.h>

OI::OI() {
  // Process operator interface input here.
  driveStick.reset(new frc::Joystick(0));
}

std::shared_ptr<frc::Joystick> OI::getDriveStick() {
  return driveStick;
}

double OI::GetLeftJoystick(){
  return driveStick->GetRawAxis(LogitechGamePad_LeftYAxis);
}

double OI::GetRightJoystick(){
  return driveStick->GetRawAxis(LogitechGamePad_RightYAxis);
}