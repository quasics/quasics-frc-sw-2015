// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IncrementLinearActuator.h"

IncrementLinearActuator::IncrementLinearActuator(Shooter* shooter):shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooter});
}

// Called when the command is initially scheduled.
void IncrementLinearActuator::Initialize() {
  shooter->IncrementPosition();
}

// Called repeatedly when this Command is scheduled to run
void IncrementLinearActuator::Execute() {}

// Called once the command ends or is interrupted.
void IncrementLinearActuator::End(bool interrupted) {}

// Returns true when the command should end.
bool IncrementLinearActuator::IsFinished() {
  return true;
}
