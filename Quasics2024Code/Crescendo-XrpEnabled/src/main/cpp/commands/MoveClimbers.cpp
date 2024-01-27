// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveClimbers.h"

MoveClimbers::MoveClimbers(Climber* climber, bool extending)
    : m_climber(climber), m_extending(extending) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_climber);
}

// Called when the command is initially scheduled.
void MoveClimbers::Initialize() {
  if (m_extending) {
    m_climber->StartExtending();
  } else {
    m_climber->StartRetracting();
  }
  m_climber->EnableBraking(true);
}

// Called repeatedly when this Command is scheduled to run
void MoveClimbers::Execute() {
  if (m_extending) {
    m_climber->StartExtending();
  } else {
    m_climber->StartRetracting();
  }
  m_climber->EnableBraking(true);
}

// Called once the command ends or is interrupted.
void MoveClimbers::End(bool interrupted) {
  m_climber->Stop();
  m_climber->EnableBraking(true);
}

// Returns true when the command should end.
bool MoveClimbers::IsFinished() {
  if (m_extending) {
    if (m_climber->IsFullyExtended()) {
      return true;
    }
    return false;
  } else {
    if (m_climber->IsFullyRetracted()) {
      return true;
    }
    return false;
  }
  return false;
}
