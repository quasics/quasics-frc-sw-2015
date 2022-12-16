// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "commands/rotate.h"

#include <iostream>

rotate::rotate(DriveBase* driveBase, int degrees,
               double percentSpeed, bool turnLeft)
    : m_driveBase(driveBase),
      m_degrees(degrees),
      m_percentSpeed(percentSpeed),
      m_turnLeft(turnLeft) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_driveBase);
}

// Called when the command is initially scheduled.
void rotate::Initialize() {
  m_driveBase->ResetGyro();
  if (m_turnLeft) {
    m_driveBase->TankDrive(m_percentSpeed, -m_percentSpeed);
  }
  else {
    m_driveBase->TankDrive(-m_percentSpeed, m_percentSpeed);
  }
}

// Called repeatedly when this Command is scheduled to run
void rotate::Execute() {
  if (m_turnLeft) {
    m_driveBase->TankDrive(m_percentSpeed, -m_percentSpeed);
  }
  else {
    m_driveBase->TankDrive(-m_percentSpeed, m_percentSpeed);
  }
}

// Called once the command ends or is interrupted.
void rotate::End(bool interrupted) {
  m_driveBase->Stop();
}

/*
  move 1 meter
  move on 90 degree arc of circle, turning left
  move 1 meter
  move on 90 degree arc of circle, turning left
  move 1 meter
  move on 90 degree arc of circle, turning right
  move 1 meter
  move on 90 degree arc of circle, turning right
  move 1 meter
  move on 90 degree arc of circle, turning right
  move 1 meter
  move on 90 degree arc of circle, turning right
  move 1 meter
  move on 90 degree arc of circle, turning left
  move 1 meter
  move on 90 degree arc of circle, turning left

*/

// Returns true when the command should end.
bool rotate::IsFinished() {
  int degreesRotated = m_driveBase->GetHeading();
  return (degreesRotated > 90 || degreesRotated < -90);
}
