// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RetractClimber.h"

RetractClimber::RetractClimber(Climber* climber) : m_climber(climber) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_climber);
}

// Called when the command is initially scheduled.
void RetractClimber::Initialize() {
  m_climber->StartRetracting();
}

// Called once the command ends or is interrupted.
void RetractClimber::End(bool interrupted) {
  m_climber->EnableBraking(true);
  m_climber->Stop();
}

// Returns true when the command should end.
bool RetractClimber::IsFinished() {
  return false;
  // return m_climber->IsFullyRetracted();
}
