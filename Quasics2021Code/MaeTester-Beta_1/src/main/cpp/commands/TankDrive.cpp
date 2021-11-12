// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"

TankDrive::TankDrive(Drivebase* drivebase, std::function<double()> right,
                     std::function<double()> left)
    : drivebase(drivebase), right(right), left(left) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivebase});
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
  drivebase->SetMotorSpeed(left(), right());
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {
  drivebase->Stop();
}

// Returns true when the command should end.
bool TankDrive::IsFinished() {
  return false;
}
