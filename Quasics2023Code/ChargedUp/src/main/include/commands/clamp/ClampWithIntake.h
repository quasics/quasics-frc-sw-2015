// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeClamp.h"

/*
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */

// This is an Outdated Command since Clamp Will not be used in this year's bot
// Runs the Motor on the clamp to close upon the game piece
class ClampWithIntake
    : public frc2::CommandHelper<frc2::CommandBase, ClampWithIntake> {
 public:
  ClampWithIntake(IntakeClamp* clamp, double power);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeClamp* m_IntakeClamp;
  const double m_clampPower;
};
