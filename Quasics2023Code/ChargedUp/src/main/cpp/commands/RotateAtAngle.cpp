// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateAtAngle.h"
#include <units/angle.h>


RotateAtAngle::RotateAtAngle(Drivebase* drivebase, double percentSpeed, units::degree_t angle) 
 : m_drivebase(drivebase), m_percentSpeed(percentSpeed), m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drivebase);
}

// Called when the command is initially scheduled.
void RotateAtAngle::Initialize() {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->TankDrive(-m_percentSpeed, -m_percentSpeed);
  m_startAngle = m_drivebase->GetAngle();
}

// Called repeatedly when this Command is scheduled to run
void RotateAtAngle::Execute() {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->TankDrive(-m_percentSpeed, m_percentSpeed);
}

// Called once the command ends or is interrupted.
void RotateAtAngle::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool RotateAtAngle::IsFinished() {
  units::degree_t currentAngle = m_drivebase->GetAngle();
  return (currentAngle >= m_angle + m_startAngle);
}
