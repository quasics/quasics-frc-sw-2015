// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExtendIntake.h"

ExtendIntake::ExtendIntake(Pneumatics*pneumatics): pneumatics(pneumatics) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(pneumatics);
}

// Called when the command is initially scheduled.
void ExtendIntake::Initialize() {
  pneumatics->ExtendSolenoid();
}

// Called repeatedly when this Command is scheduled to run
void ExtendIntake::Execute() {}

// Called once the command ends or is interrupted.
void ExtendIntake::End(bool interrupted) {}

// Returns true when the command should end.
bool ExtendIntake::IsFinished() {
  return false;
}
