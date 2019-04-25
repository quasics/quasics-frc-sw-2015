/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include <frc/commands/Subsystem.h>
#include <frc/SpeedController.h>

class TestMotor : public frc::Command {
 private:
  frc::SpeedController & speedController;
  double power;
 public:
   TestMotor(frc::Subsystem &subsystem, frc::SpeedController& speedController, double power);
   void Initialize() override;
   void End() override;
   void Interrupted() override;
   bool IsFinished() override;
};
