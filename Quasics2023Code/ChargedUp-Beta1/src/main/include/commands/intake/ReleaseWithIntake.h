// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeClamp.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 */

// Outdated Command this intake will not be used on the robot
// Runs the clamp motor backwards to allow the robot to release the game piece
// held with a clamp
class ReleaseWithIntake
    : public frc2::CommandHelper<frc2::Command, ReleaseWithIntake> {
 public:
  ReleaseWithIntake(IntakeClamp* clamp, double power);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeClamp* m_IntakeClamp;
  const double m_clampPower;
};
