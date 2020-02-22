/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveBase.h"

/**
 * Simple command to run the drive base at a given speed(s), for a specified
 * duration.
 */
class MoveForTimeCommand
    : public frc2::CommandHelper<frc2::CommandBase, MoveForTimeCommand> {
 public:
  MoveForTimeCommand(DriveBase* drivebase, double duration, double power)
      : MoveForTimeCommand(drivebase, duration, power, power) {
  }
  MoveForTimeCommand(DriveBase* drivebase, double duration, double leftPower,
                     double rightPower)
      : drivebase(drivebase),
        duration(duration),
        leftPower(leftPower),
        rightPower(rightPower) {
    AddRequirements(drivebase);
  }

  void Initialize() override {
    timer.Reset();
    timer.Start();
    drivebase->SetMotorPower(leftPower, rightPower);
  }

  void End(bool interrupted) override {
    timer.Stop();
    drivebase->Stop();
  }

  bool IsFinished() override {
    return timer.HasPeriodPassed(duration);
  }

 private:
  DriveBase* const drivebase;
  const double duration;
  const double leftPower;
  const double rightPower;
  frc::Timer timer;
};
