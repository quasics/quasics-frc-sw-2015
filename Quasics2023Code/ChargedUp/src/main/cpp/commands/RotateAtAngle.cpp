// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateAtAngle.h"
#include <units/angle.h>
#include <iostream>

RotateAtAngle::RotateAtAngle(Drivebase* drivebase, double percentSpeed, units::degree_t angle) 
 : m_drivebase(drivebase), m_percentSpeed(percentSpeed), m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drivebase);
}

// Called when the command is initially scheduled.
void RotateAtAngle::Initialize() {
  m_drivebase->SetBrakingMode(true);
  if (m_angle >= 0_deg)
    m_drivebase->TankDrive(-m_percentSpeed, m_percentSpeed);
  else 
    m_drivebase->TankDrive(m_percentSpeed, -m_percentSpeed);
  m_startAngle = m_drivebase->GetAngle();
}

// Called repeatedly when this Command is scheduled to run
void RotateAtAngle::Execute() {
  double newSpeed = m_percentSpeed;
  m_drivebase->SetBrakingMode(true);
  units::degree_t currentPosition = m_drivebase->GetAngle();

  units::degree_t degreesLeft = (m_startAngle + m_angle) - currentPosition;
  
  if (m_angle >= 0_deg) {
    if (degreesLeft < 60_deg) {
      newSpeed = 0.25;
    }
    std::cout << "Degrees left: " << degreesLeft.value() << ", Speed: " << newSpeed << std::endl;
    m_drivebase->TankDrive(-newSpeed, newSpeed);
  }

  else {
    if (degreesLeft < -60_deg) {
      newSpeed = 0.25;
    }
    std::cout << "Degrees left: " << degreesLeft.value() << ", Speed: " << newSpeed << std::endl;
    m_drivebase->TankDrive(newSpeed, -newSpeed);
  }


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
