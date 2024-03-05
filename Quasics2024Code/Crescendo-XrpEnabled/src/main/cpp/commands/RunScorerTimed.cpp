// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunScorerTimed.h"

RunScorerTimed::RunScorerTimed(PivotScorer &scorer, double scorerSpeed,
                               units::second_t time, bool up)
    : m_scorer(scorer),
      m_scorerSpeed(up ? -std::abs(scorerSpeed) : std::abs(scorerSpeed)),
      m_time(time),
      m_up(up) {
  AddRequirements(&m_scorer);
}

// Called when the command is initially scheduled.
void RunScorerTimed::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_scorer.SetScorerSpeed(m_scorerSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunScorerTimed::Execute() {
  m_scorer.SetScorerSpeed(m_scorerSpeed);
}

// Called once the command ends or is interrupted.
void RunScorerTimed::End(bool interrupted) {
  m_scorer.Stop();
}

// Returns true when the command should end.
bool RunScorerTimed::IsFinished() {
  return m_stopWatch.HasElapsed(m_time);
}
