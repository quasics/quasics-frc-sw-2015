// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunAmpScorer.h"

RunAmpScorer::RunAmpScorer(AmpScorer &ampScorer, double ampScorerSpeed,
                           bool extending)
    : m_ampScorer(ampScorer),
      m_ampScorerSpeed(extending ? -std::abs(ampScorerSpeed)
                                 : std::abs(ampScorerSpeed)),
      m_extending(extending) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&m_ampScorer);
}

// Called when the command is initially scheduled.
void RunAmpScorer::Initialize() {
  m_ampScorer.SetAmpScorerSpeed(m_ampScorerSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunAmpScorer::Execute() {
  m_ampScorer.SetAmpScorerSpeed(m_ampScorerSpeed);
}

// Called once the command ends or is interrupted.
void RunAmpScorer::End(bool interrupted) {
  m_ampScorer.Stop();
}
