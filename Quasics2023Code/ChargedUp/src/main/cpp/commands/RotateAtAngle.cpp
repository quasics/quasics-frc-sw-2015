// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateAtAngle.h"

#include <units/angle.h>

#include <iostream>

RotateAtAngle::RotateAtAngle(Drivebase* drivebase, double percentSpeed,
                             units::degree_t angle)
    : m_drivebase(drivebase),
      m_percentSpeed((percentSpeed > 0) ? percentSpeed : -percentSpeed),
      // robot seems to overshoot about 2 degrees even after slowing down
      m_angle((percentSpeed > 0) ? (angle) : (-angle)) {
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
  if (m_percentSpeed < 0.25) {
    if (m_angle > 0_deg)
      m_drivebase->TankDrive(-m_percentSpeed, m_percentSpeed);
    else
      m_drivebase->TankDrive(m_percentSpeed, -m_percentSpeed);
    return;
  }

  m_drivebase->SetBrakingMode(true);
  units::degree_t currentPosition = m_drivebase->GetAngle();

  units::degree_t degreesLeft = (m_startAngle + m_angle) - currentPosition;
  units::degree_t degreesLeftWhenSlowDown = 90_deg;

  if (m_angle >= 0_deg) {
    if (degreesLeft < degreesLeftWhenSlowDown &&
        m_percentSpeed * m_multiplier > 0.25) {
      m_multiplier *= .99;
    }
    m_drivebase->TankDrive(-m_percentSpeed * m_multiplier,
                           m_percentSpeed * m_multiplier);
  }

  else {
    if (-degreesLeft < degreesLeftWhenSlowDown &&
        m_percentSpeed * m_multiplier > 0.25) {
      m_multiplier *= .99;
    }
    m_drivebase->TankDrive(m_percentSpeed * m_multiplier,
                           -m_percentSpeed * m_multiplier);
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
