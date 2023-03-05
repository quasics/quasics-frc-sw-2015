// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivebase.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 * Tank drives using the speeds returned by leftSpeedFunction() and
 * rightSpeedFunction(). Used by joysticks in RobotContainer
 *
 * CODE_REVIEW(josh/ethan): Please update the documentation for this command,
 * including the comments above (which indicate that this is "an example
 * command"), so that it's clear what the command does, and how it is expected
 * to be used.
 */
class TankDrive : public frc2::CommandHelper<frc2::CommandBase, TankDrive> {
 public:
  TankDrive(Drivebase* drivebase, std::function<double()> leftSpeedFunction,
            std::function<double()> rightSpeedFunction);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  void UpdateSpeeds();

  Drivebase* m_drivebase;
  std::function<double()> m_leftSpeedFunction;
  std::function<double()> m_rightSpeedFunction;

  // working with logitech controller
};
