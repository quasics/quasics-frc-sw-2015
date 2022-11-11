// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "commands/rotate.h"

#include <iostream>

rotate::rotate(DriveBase* driveBase, int degrees,
               double percentSpeed)
    : m_driveBase(driveBase),
      m_degrees(degrees),
      m_percentSpeed(percentSpeed) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_driveBase);
}

// Called when the command is initially scheduled.
void rotate::Initialize() {
  std::cout << "Initializing" << std::endl;
  m_driveBase->ResetGyro();
  m_driveBase->TankDrive(-m_percentSpeed, m_percentSpeed);
}

// Called repeatedly when this Command is scheduled to run
void rotate::Execute() {
  std::cout << "Execute" << std::endl;

  m_driveBase->TankDrive(-m_percentSpeed, m_percentSpeed);
}

// Called once the command ends or is interrupted.
void rotate::End(bool interrupted) {
  std::cout << "End" << std::endl;
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
