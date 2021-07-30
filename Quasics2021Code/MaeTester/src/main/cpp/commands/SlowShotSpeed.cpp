// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SlowShotSpeed.h"
/*
SlowShotSpeed::SlowShotSpeed(Shooter* shooter, double power) 
    : shooter(shooter), power(power) {
  // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(shooter);
}

// Called when the command is initially scheduled.
void SlowShotSpeed::Initialize() {
  shooter->setSpeed(power);  // makes ball go yeet
}

// Called repeatedly when this Command is scheduled to run
void SlowShotSpeed::Execute() {
  shooter->setSpeed(power); //makes yeet wheel keep spinning
}

// Called once the command ends or is interrupted.
void SlowShotSpeed::End(bool interrupted) {
  shooter->Stop();
}

// Returns true when the command should end.
bool SlowShotSpeed::IsFinished() {
  return false;
}
