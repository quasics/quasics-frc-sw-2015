// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Drivebase.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TankDrive
    : public frc2::CommandHelper<frc2::Command, TankDrive> {
 public:
  TankDrive(Drivebase*drivebase, std::function<double()> right, std::function<double()> left);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
Drivebase*drivebase;
std::function<double()> right;
std::function<double()> left;
};
