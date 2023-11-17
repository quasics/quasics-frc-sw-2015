// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"

TankDrive::TankDrive(DriveBase* drivebase, std::function<double()> leftSpeedFunction, std::function<double()> rightSpeedFunction): m_drivebase(drivebase), m_leftSpeedFunction(leftSpeedFunction), m_rightSpeedFunction(rightSpeedFunction) {
AddRequirements(drivebase);
SetName("TankDrive");  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {
  UpdateSpeeds();
}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
  UpdateSpeeds();
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool TankDrive::IsFinished() {
  return false;
}

void TankDrive::UpdateSpeeds() {
  m_drivebase->TankDrive(m_leftSpeedFunction(), m_rightSpeedFunction());
}
