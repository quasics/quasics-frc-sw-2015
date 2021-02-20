// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunShootingMotor.h"

RunShootingMotor::RunShootingMotor(Shooter*shooter)
 : shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void RunShootingMotor::Initialize() {
  shooter->setShootingMotor(.75);
}

// Called repeatedly when this Command is scheduled to run
void RunShootingMotor::Execute() {}

// Called once the command ends or is interrupted.
void RunShootingMotor::End(bool interrupted) {
  shooter->stopShootingMotor();
}

// Returns true when the command should end.
bool RunShootingMotor::IsFinished() {
  return false;
}
