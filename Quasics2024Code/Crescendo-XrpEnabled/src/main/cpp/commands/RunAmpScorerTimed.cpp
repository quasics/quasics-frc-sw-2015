// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunAmpScorerTimed.h"

RunAmpScorerTimed::RunAmpScorerTimed(AmpScorer &ampScorer,
                                     double ampScorerSpeed,
                                     units::second_t time, bool extending)
    : m_ampScorer(ampScorer),
      m_ampScorerSpeed(extending ? -std::abs(ampScorerSpeed)
                                 : std::abs(ampScorerSpeed)),
      m_time(time),
      m_extending(extending) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&m_ampScorer);
}

// Called when the command is initially scheduled.
void RunAmpScorerTimed::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_ampScorer.SetAmpScorerSpeed(m_ampScorerSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunAmpScorerTimed::Execute() {
  m_ampScorer.SetAmpScorerSpeed(m_ampScorerSpeed);
}

// Called once the command ends or is interrupted.
void RunAmpScorerTimed::End(bool interrupted) {
  m_ampScorer.Stop();
}

// Returns true when the command should end.
bool RunAmpScorerTimed::IsFinished() {
  return m_stopWatch.HasElapsed(m_time);
}
