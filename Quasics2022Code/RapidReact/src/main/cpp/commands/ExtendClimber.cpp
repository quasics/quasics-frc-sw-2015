// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExtendClimber.h"

ExtendClimber::ExtendClimber(Climber* climber) : m_climber(climber) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements(m_climber);
}

// Called when the command is initially scheduled.
void ExtendClimber::Initialize() {
  m_climber->StartExtending();
}

// Called once the command ends or is interrupted.
void ExtendClimber::End(bool interrupted) {
  m_climber->EnableBraking(true);
}

// Returns true when the command should end.
bool ExtendClimber::IsFinished() {
  return false;
  // return m_climber->IsFullyExtended();
}
