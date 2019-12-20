/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef OI_H
#define OI_H

#include "frc/WPILib.h"

class OI {
private:
      std::shared_ptr<frc::Joystick> driveStick;
public:
  std::shared_ptr<frc::Joystick> getDriveStick();
  double GetLeftJoystick();
  double GetRightJoystick();
  OI();
};
#endif