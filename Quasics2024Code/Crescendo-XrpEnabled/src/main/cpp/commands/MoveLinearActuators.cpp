// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveLinearActuators.h"

MoveLinearActuators::MoveLinearActuators(Shooter& shooter, bool extending)
    : m_shooter(shooter), m_extending(extending) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&m_shooter);
}

// Called when the command is initially scheduled.
void MoveLinearActuators::Initialize() {
  if (m_extending) {
    m_shooter.ExtendLinearActuators();
  } else {
    m_shooter.RetractLinearActuators();
  }
}

// Called repeatedly when this Command is scheduled to run
void MoveLinearActuators::Execute() {
}

// Called once the command ends or is interrupted.
void MoveLinearActuators::End(bool interrupted) {
}

// Returns true when the command should end.
bool MoveLinearActuators::IsFinished() {
  return false;
}
