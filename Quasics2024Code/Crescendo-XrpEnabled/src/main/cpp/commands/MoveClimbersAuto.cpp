// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveClimbersAuto.h"

MoveClimbersAuto::MoveClimbersAuto(Climber& climber, bool extending)
    : m_climber(climber), m_extending(extending) {
  AddRequirements(&m_climber);
}

// Called when the command is initially scheduled.
void MoveClimbersAuto::Initialize() {
  double leftRevolutions = m_climber.getLeftRevolutions();
  double rightRevolutions = m_climber.getRightRevolutions();
  if (m_extending) {
    if (leftRevolutions > -5) {
      // the bool is asking if its the left climber we want
      m_climber.ExtendOneClimber(true);
    }
    if (rightRevolutions > -5) {
      m_climber.ExtendOneClimber(false);
    }
  } else {
    if (leftRevolutions < 0) {
      // the bool is asking if its the left climber we want
      m_climber.RetractOneClimber(true);
    }
    if (rightRevolutions < 0) {
      m_climber.RetractOneClimber(false);
    }
  }
}

// Called repeatedly when this Command is scheduled to run
void MoveClimbersAuto::Execute() {
  double leftRevolutions = m_climber.getLeftRevolutions();
  double rightRevolutions = m_climber.getRightRevolutions();
  if (m_extending) {
    if (leftRevolutions > -5) {
      // the bool is asking if its the left climber we want
      m_climber.ExtendOneClimber(true);
    }
    if (rightRevolutions > -5) {
      m_climber.ExtendOneClimber(false);
    }
  } else {
    if (leftRevolutions < 0) {
      // the bool is asking if its the left climber we want
      m_climber.RetractOneClimber(true);
    }
    if (rightRevolutions < 0) {
      m_climber.RetractOneClimber(false);
    }
  }
}

// Called once the command ends or is interrupted.
void MoveClimbersAuto::End(bool interrupted) {
  m_climber.Stop();
}

// Returns true when the command should end.
bool MoveClimbersAuto::IsFinished() {
  if (m_extending) {
    if (m_climber.getLeftRevolutions() < -5 &&
        m_climber.getRightRevolutions() < -5) {
      return true;
    }
  }
  return false;
}
