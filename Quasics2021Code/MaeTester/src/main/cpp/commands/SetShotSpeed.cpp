// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetShotSpeed.h"

SetShotSpeed::SetShotSpeed(Shooter* shooter, double power)
    : shooter(shooter), power(power) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void SetShotSpeed::Initialize() {
  shooter->SetSpeed(power);  // makes ball go yeet
}

// Called repeatedly when this Command is scheduled to run
void SetShotSpeed::Execute() {
  shooter->SetSpeed(power); //makes yeet wheel keep spinning
}

// Called once the command ends or is interrupted.
void SetShotSpeed::End(bool interrupted) {
  shooter->Stop();
}

// Returns true when the command should end.
bool SetShotSpeed::IsFinished() {
  return false;
}
