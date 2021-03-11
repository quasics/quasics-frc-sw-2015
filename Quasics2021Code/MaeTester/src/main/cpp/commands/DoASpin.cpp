// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DoASpin.h"

DoASpin::DoASpin(Drivebase* drivebase) : drivebase(drivebase) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void DoASpin::Initialize() {
  drivebase->SetMotorSpeed(.3, .3);
}

// Called repeatedly when this Command is scheduled to run
void DoASpin::Execute() {
}

// Called once the command ends or is interrupted.
void DoASpin::End(bool interrupted) {
  drivebase->Stop();
}

// Returns true when the command should end.
bool DoASpin::IsFinished() {
  return false;
}
