// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Square.h"

Square::Square() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Square::Initialize() {}
// turn on all 4 motors

// Called repeatedly when this Command is scheduled to run
void Square::Execute() {}
//move a certain distance
//turn front motor for a certain time
//repeat 4 times
// Called once the command ends or is interrupted.
void Square::End(bool interrupted) {}
//turn off all 4 motors

// Returns true when the command should end.
bool Square::IsFinished() {
  //when all 4 motors are off return true
  return false;
}
