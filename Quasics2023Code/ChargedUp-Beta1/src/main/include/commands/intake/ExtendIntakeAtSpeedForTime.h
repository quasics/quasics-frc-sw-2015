// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeDeployment.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 */

// Runs the intake deployment motor at a speed for a certain time to allow the
// robot to get game pieces

class ExtendIntakeAtSpeedForTime
    : public frc2::CommandHelper<frc2::Command,
                                 ExtendIntakeAtSpeedForTime> {
 public:
  ExtendIntakeAtSpeedForTime(IntakeDeployment* IntakeDeployment, double speed,
                             units::second_t time);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeDeployment* m_intakeDeployment;
  const double intakeSpeed;
  const units::second_t m_time;
  frc::Timer m_stopWatch;
};
