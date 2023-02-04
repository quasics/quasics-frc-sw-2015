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
  multiplier = 1.0;
  m_drivebase->SetBrakingMode(true);
  if (m_angle >= 0_deg)
    m_drivebase->TankDrive(-m_percentSpeed * multiplier, m_percentSpeed * multiplier);
  else 
    m_drivebase->TankDrive(m_percentSpeed * multiplier, -m_percentSpeed * multiplier);
  m_startAngle = m_drivebase->GetAngle();
}

// Called repeatedly when this Command is scheduled to run
void RotateAtAngle::Execute() {
  m_drivebase->SetBrakingMode(true);
  units::degree_t currentPosition = m_drivebase->GetAngle();
  if (currentPosition > ((m_startAngle + m_angle) * 0.5) &&
        (m_percentSpeed * multiplier > 0.26)) {
    multiplier = multiplier * 0.95;
  }
  if (m_angle >= 0_deg)
    m_drivebase->TankDrive(-m_percentSpeed * multiplier, m_percentSpeed * multiplier);
  else 
    m_drivebase->TankDrive(m_percentSpeed * multiplier, -m_percentSpeed * multiplier);
}

// Called once the command ends or is interrupted.
void RotateAtAngle::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool RotateAtAngle::IsFinished() {
  // positive angle turns left, negative angle turns right
  units::degree_t currentAngle = m_drivebase->GetAngle();
  if (m_angle >= 0_deg) 
    return (currentAngle >= m_angle + m_startAngle);
  else 
    return (currentAngle <= m_angle + m_startAngle);
}
