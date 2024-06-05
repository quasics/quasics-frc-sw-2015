// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunTransitionRollerTimed.h"

RunTransitionRollerTimed::RunTransitionRollerTimed(
    TransitionRoller &transitionRoller, double transitionSpeed,
    units::second_t time, bool transitionTakingIn)
    : m_transitionRoller(transitionRoller),
      m_transitionSpeed(transitionTakingIn ? -std::abs(transitionSpeed)
                                           : std::abs(transitionSpeed)),
      m_time(time),
      m_transitionTakingIn(transitionTakingIn) {
  AddRequirements(&m_transitionRoller);
}

// Called when the command is initially scheduled.
void RunTransitionRollerTimed::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_transitionRoller.SetTransitionRollerSpeed(m_transitionSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunTransitionRollerTimed::Execute() {
  m_transitionRoller.SetTransitionRollerSpeed(m_transitionSpeed);
}

// Called once the command ends or is interrupted.
void RunTransitionRollerTimed::End(bool interrupted) {
  m_transitionRoller.Stop();
}

// Returns true when the command should end.
bool RunTransitionRollerTimed::IsFinished() {
  return m_stopWatch.HasElapsed(m_time);
}
