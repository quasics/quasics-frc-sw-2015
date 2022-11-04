// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "commands/rotate.h"

#include <iostream>

rotate::rotate(DriveBase* driveBase, units::meter_t degrees,
               double percentSpeed)
    : m_driveBase(driveBase),
      m_degrees(degrees),
      m_percentSpeed(percentSpeed),
      m_startPosition(0) {
  std::cerr << "test" << std::endl;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_driveBase);
}

// Called when the command is initially scheduled.
void rotate::Initialize() {
  std::cout << "Initializing" << std::endl;
  m_startPosition = m_driveBase->GetLeftDistance();
  m_driveBase->TankDrive(m_percentSpeed, -m_percentSpeed);
}

// Called repeatedly when this Command is scheduled to run
void rotate::Execute() {
  std::cout << "Execute" << std::endl;

  m_driveBase->TankDrive(m_percentSpeed, -m_percentSpeed);
}

// Called once the command ends or is interrupted.
void rotate::End(bool interrupted) {
  std::cout << "End" << std::endl;
  m_driveBase->Stop();
}

// Returns true when the command should end.
bool rotate::IsFinished() {
  const units::meter_t currentPosition = m_driveBase->GetLeftDistance();

  return (currentPosition >= (m_startPosition + m_degrees));
}
