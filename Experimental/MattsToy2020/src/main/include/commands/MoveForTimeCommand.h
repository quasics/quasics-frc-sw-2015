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
  /**
   * Constructor.
   *
   * @param drivebase  pointer to the drive base subsystem to be used while the
   *                   command is running
   * @param duration   time (in seconds) that the the drive base should move
   *                   while the command is running
   * @param power      power level (-1.0 to +1.0) to be applied to both left and
   *                   right wheels while the command is running
   */
  MoveForTimeCommand(DriveBase* drivebase, double duration, double power)
      : MoveForTimeCommand(drivebase, duration, power, power) {
  }

  /**
   * Constructor.
   *
   * @param drivebase  pointer to the drive base subsystem to be used while the
   *                   command is running
   * @param duration   time (in seconds) that the the drive base should move
   *                   while the command is running
   * @param leftPower  power level (-1.0 to +1.0) to be applied to left wheels
   * @param rightPower power level (-1.0 to +1.0) to be applied to right wheels
   */
  MoveForTimeCommand(DriveBase* drivebase, double duration, double leftPower,
                     double rightPower)
      : drivebase(drivebase),
        duration(duration),
        leftPower(leftPower),
        rightPower(rightPower) {
    AddRequirements(drivebase);
  }

  /**
   * Overrides Initialize() from base to start the motors and time running.
   */
  void Initialize() override {
    timer.Reset();
    timer.Start();
    drivebase->SetMotorPower(leftPower, rightPower);
  }

  /**
   * Overrides End() from base to stop the motors.
   */
  void End(bool interrupted) override {
    timer.Stop();
    drivebase->Stop();
  }

  /**
   * Returns true iff the command has been running for the specified duration.
   */
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
