// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunTransitionRoller.h"

RunTransitionRoller::RunTransitionRoller(TransitionRoller &transitionRoller,
                                         double transitionSpeed,
                                         bool transitionTakingIn)
    : m_transitionRoller(transitionRoller),
      m_transitionSpeed(transitionTakingIn ? -std::abs(transitionSpeed)
                                           : std::abs(transitionSpeed)),
      m_transitionTakingIn(transitionTakingIn) {
  AddRequirements(&m_transitionRoller);
}

// Called when the command is initially scheduled.
void RunTransitionRoller::Initialize() {
  m_transitionRoller.SetTransitionRollerSpeed(m_transitionSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RunTransitionRoller::Execute() {
  m_transitionRoller.SetTransitionRollerSpeed(m_transitionSpeed);
}

// Called once the command ends or is interrupted.
void RunTransitionRoller::End(bool interrupted) {
  m_transitionRoller.Stop();
}
