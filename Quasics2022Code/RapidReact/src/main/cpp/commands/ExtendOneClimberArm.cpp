// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExtendOneClimberArm.h"

ExtendOneClimberArm::ExtendOneClimberArm(Climber* climber, bool isLeftClimber)
    : m_climber(climber), m_isLeftClimber(isLeftClimber) {
  AddRequirements(m_climber);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ExtendOneClimberArm::Initialize() {
  m_climber->ExtendOneClimber(m_isLeftClimber);
}

// Called once the command ends or is interrupted.
void ExtendOneClimberArm::End(bool interrupted) {
  m_climber->EnableBraking(true);
  m_climber->Stop();
}

// Returns true when the command should end.
bool ExtendOneClimberArm::IsFinished() {
  return false;
}
