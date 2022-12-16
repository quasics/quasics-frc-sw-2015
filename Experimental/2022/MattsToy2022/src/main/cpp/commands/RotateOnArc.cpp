// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "commands/RotateOnArc.h"

#include <iostream>

RotateOnArc::RotateOnArc(DriveBase* driveBase, int degrees,
               double percentSpeed, bool turnLeft)
    : m_driveBase(driveBase),
      m_degrees(degrees),
      m_percentSpeed(percentSpeed),
      m_turnLeft(turnLeft) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_driveBase);
}

// Called when the command is initially scheduled.
void RotateOnArc::Initialize() {
  std::cout << "Initializing" << std::endl;
  m_driveBase->ResetGyro();
  if (m_turnLeft) {
    m_driveBase->TankDrive(m_percentSpeed, -0.60*m_percentSpeed);
  }
  else { 
    m_driveBase->TankDrive(-0.60*m_percentSpeed, m_percentSpeed);
  }
}

// Called repeatedly when this Command is scheduled to run
void RotateOnArc::Execute() {
  std::cout << "Execute" << std::endl;

  if (m_turnLeft) {
    m_driveBase->TankDrive(m_percentSpeed, -0.60*m_percentSpeed);
  }
  else { 
    m_driveBase->TankDrive(-0.60*m_percentSpeed, m_percentSpeed);
  }
}

// Called once the command ends or is interrupted.
void RotateOnArc::End(bool interrupted) {
  std::cout << "End" << std::endl;
  m_driveBase->Stop();
}

// Returns true when the command should end.
bool RotateOnArc::IsFinished() {
  int degreesRotated = m_driveBase->GetHeading();
  return (degreesRotated > 90 || degreesRotated < -90);
}
