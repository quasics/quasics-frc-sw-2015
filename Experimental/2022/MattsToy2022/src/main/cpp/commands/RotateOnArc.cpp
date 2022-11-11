// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateOnArc.h"

RotateOnArc::RotateOnArc(
    DriveBase* driveBase,
    bool turnLeft,
    double percentSpeed) :
  m_driveBase(driveBase),
  m_turnLeft(turnLeft),
  m_percentSpeed(percentSpeed),
  m_degreesTurned(0)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_driveBase);
}

// Called when the command is initially scheduled.
void RotateOnArc::Initialize() {
    m_driveBase->ResetGyro();
}

// Called repeatedly when this Command is scheduled to run
void RotateOnArc::Execute() {

}

// Called once the command ends or is interrupted.
void RotateOnArc::End(bool interrupted) {
  m_driveBase->Stop();
}

// Returns true when the command should end.
bool RotateOnArc::IsFinished() {
  int degreesRotated = m_driveBase->GetHeading();
  return (degreesRotated > 90);
}
