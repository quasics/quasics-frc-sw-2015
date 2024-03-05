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
    m_climber.resetRevolutions();
    if (leftRevolutions > -3) {
      // the bool is asking if its the left climber we want
      m_climber.ExtendOneClimber(true);
    }
    if (rightRevolutions > -3) {
      m_climber.ExtendOneClimber(false);
    }
  } else {
    m_climber.setRevolutions();
    if (leftRevolutions <= 0) {
      // the bool is asking if its the left climber we want
      m_climber.RetractOneClimber(true);
    }
    if (rightRevolutions <= 0) {
      m_climber.RetractOneClimber(false);
    }
  }
}

// Called repeatedly when this Command is scheduled to run
void MoveClimbersAuto::Execute() {
  double leftRevolutions = m_climber.getLeftRevolutions();
  double rightRevolutions = m_climber.getRightRevolutions();
  if (m_extending) {
    if (leftRevolutions > -3) {
      // the bool is asking if its the left climber we want
      m_climber.ExtendOneClimber(true);
    }
    if (rightRevolutions > -3) {
      m_climber.ExtendOneClimber(false);
    }
  } else {
    if (leftRevolutions <= 0) {
      // the bool is asking if its the left climber we want
      m_climber.RetractOneClimber(true);
    }
    if (rightRevolutions <= 0) {
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
    if (m_climber.getLeftRevolutions() < -3 &&
        m_climber.getRightRevolutions() < -3) {
      return true;
    }
    return false;
  } else {
    if (m_climber.getLeftRevolutions() >= 0 &&
        m_climber.getRightRevolutions() >= 0) {
      return true;
    }
    return false;
  }
  return false;
}
