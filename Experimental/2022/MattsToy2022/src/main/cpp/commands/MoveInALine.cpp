// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveInALine.h"

MoveInALine::MoveInALine(
    DriveBase* driveBase,
    units::meter_t distanceToTravel,
    double percentSpeed) :
  m_driveBase(driveBase),
  m_distanceToTravel(distanceToTravel),
  m_percentSpeed(percentSpeed),
  m_startPosition(0)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_driveBase);
}

// Called when the command is initially scheduled.
void MoveInALine::Initialize() {
  m_startPosition = m_driveBase->GetLeftDistance();
  m_driveBase->TankDrive(m_percentSpeed, m_percentSpeed);
}

// Called repeatedly when this Command is scheduled to run
void MoveInALine::Execute() {
  m_driveBase->TankDrive(m_percentSpeed, m_percentSpeed);
}

// Called once the command ends or is interrupted.
void MoveInALine::End(bool interrupted) {
  m_driveBase->Stop();
}

// Returns true when the command should end.
bool MoveInALine::IsFinished() {
  const units::meter_t currentPosition = m_driveBase->GetLeftDistance();
  if (currentPosition >= (m_startPosition + m_distanceToTravel)) {
    return true;
  }

  return false;
}
