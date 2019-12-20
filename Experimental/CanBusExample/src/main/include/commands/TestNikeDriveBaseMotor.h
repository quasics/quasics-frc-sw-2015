/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/TimedCommand.h>

#include "subsystems/NikeDriveBase.h"

class TestNikeDriveBaseMotor : public frc::TimedCommand {
 private:
  double speed;
  NikeDriveBase::Motor motor;

 public:
  explicit TestNikeDriveBaseMotor(NikeDriveBase::Motor motor, double speed,
                                  double timeout);
  void Initialize() override;
  void Execute() override;
  void End() override;
  void Interrupted() override;
};
