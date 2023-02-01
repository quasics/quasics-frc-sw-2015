// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"

TankDrive::TankDrive(DriveBase* driveBase, std::function<double()> leftSpeed, std::function<double()> rightSpeed) {
  m_driveBase = driveBase;
  m_leftSpeed = leftSpeed;
  m_rightSpeed = rightSpeed;

  // Tell the rest of the code that we use this when we're running.
  AddRequirements(m_driveBase);
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {
  m_driveBase->TankDrive(m_leftSpeed(), m_rightSpeed());
}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
  m_driveBase->TankDrive(m_leftSpeed(), m_rightSpeed());
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {
  m_driveBase->Stop();
}

// Returns true when the command should end.
bool TankDrive::IsFinished() {
  return false;
}
