// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RetractIntake.h"

RetractIntake::RetractIntake(Pneumatics*pneumatics): pneumatics(pneumatics) {
  // Use addRequirements() here to declare subsystem dependencies
  AddRequirements(pneumatics);
}

// Called when the command is initially scheduled.
void RetractIntake::Initialize() {
  pneumatics->RetractSolenoid();
}

// Called repeatedly when this Command is scheduled to run
void RetractIntake::Execute() {}

// Called once the command ends or is interrupted.
void RetractIntake::End(bool interrupted) {}

// Returns true when the command should end.
bool RetractIntake::IsFinished() {
  return false;
}
