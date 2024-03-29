// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/PivotScorer.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 * TODO: Add actual docs for this command....
 */
class RunScorerTimed
    : public frc2::CommandHelper<frc2::Command, RunScorerTimed> {
 public:
  RunScorerTimed(PivotScorer &scorer, double scorerSpeed, units::second_t time,
                 bool up);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  PivotScorer &m_scorer;
  const double m_scorerSpeed;
  const units::second_t m_time;
  frc::Timer m_stopWatch;
  const bool m_up;
};
