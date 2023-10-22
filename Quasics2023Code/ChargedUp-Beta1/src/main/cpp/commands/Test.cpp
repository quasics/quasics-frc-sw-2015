// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Test.h"

Test::Test() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Test::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Test::Execute() {}

// Called once the command ends or is interrupted.
void Test::End(bool interrupted) {}

// Returns true when the command should end.
bool Test::IsFinished() {
  return false;
}
