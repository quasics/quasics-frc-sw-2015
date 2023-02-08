// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveDistance.h"

DriveDistance::DriveDistance(DriveBase* driveBase, units::meter_t distanceMeters, double speed) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(driveBase);

  m_driveBase = driveBase;
  m_distance = distanceMeters;
  m_speed = speed;
  m_targetDistance = 0_m;
}

// Called when the command is initially scheduled.
void DriveDistance::Initialize() {
  m_targetDistance = m_distance + m_driveBase->GetLeftDistance();
  m_driveBase->TankDrive(m_speed, m_speed);
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute() {
  m_driveBase->TankDrive(m_speed, m_speed);
}

// Called once the command ends or is interrupted.
void DriveDistance::End(bool interrupted) {
  m_driveBase->Stop();
}

// Returns true when the command should end.
bool DriveDistance::IsFinished() {
  if (m_driveBase->GetLeftDistance() >= m_targetDistance) {
    return true;
  }
  
  return false;
}
