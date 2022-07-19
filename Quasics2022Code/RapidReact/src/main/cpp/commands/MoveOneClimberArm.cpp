// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveOneClimberArm.h"

MoveOneClimberArm::MoveOneClimberArm(Climber* climber, bool isLeftClimber,
                                     bool extend)
    : m_climber(climber), m_isLeftClimber(isLeftClimber), m_extend(extend) {
  AddRequirements(m_climber);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void MoveOneClimberArm::Initialize() {
  if (m_extend == true) {
    m_climber->ExtendOneClimber(m_isLeftClimber);
  } else {
    m_climber->RetractOneClimber(m_isLeftClimber);
  }
}

// Called once the command ends or is interrupted.
void MoveOneClimberArm::End(bool interrupted) {
  m_climber->EnableBraking(true);
  m_climber->Stop();
}

// Returns true when the command should end.
bool MoveOneClimberArm::IsFinished() {
  return false;
}
