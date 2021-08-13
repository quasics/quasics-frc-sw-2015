// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TriggerDrivenShootingCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 TriggerDrivenShootingCommand> {
 public:
  TriggerDrivenShootingCommand(Shooter* shooter, double highSpeed,
                               double lowSpeed,
                               std::function<bool()> runHighSpeed,
                               std::function<bool()> runLowSpeed);

  void Execute() override;

  void End(bool interrupted) override;

 private:
  Shooter* shooter;
  double highSpeed;
  double lowSpeed;
  std::function<bool()> runHighSpeed;
  std::function<bool()> runLowSpeed;
};
