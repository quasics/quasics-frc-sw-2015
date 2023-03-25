// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
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

// This is an outdated command since this intake will not be used on this year's
// bot Runs the motor to clamp down upon the game piece at a designated power
// for a designated time
class ClampWithIntakeAtSpeedForTime
    : public frc2::CommandHelper<frc2::CommandBase,
                                 ClampWithIntakeAtSpeedForTime> {
 public:
  ClampWithIntakeAtSpeedForTime(IntakeClamp* clamp, double power,
                                units::second_t time);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeClamp* m_IntakeClamp;
  const double m_clampPower;
  const units::second_t m_time;
  frc::Timer m_stopWatch;
};
