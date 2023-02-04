// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DecrementLinearActuator.h"

DecrementLinearActuator::DecrementLinearActuator(Shooter* shooter):shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void DecrementLinearActuator::Initialize() {
  shooter->DecrementPosition();
}

// Called repeatedly when this Command is scheduled to run
void DecrementLinearActuator::Execute() {}

// Called once the command ends or is interrupted.
void DecrementLinearActuator::End(bool interrupted) {}

// Returns true when the command should end.
bool DecrementLinearActuator::IsFinished() {
  return true;
}
