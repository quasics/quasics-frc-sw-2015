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
      // -5 and +5 added because robot generally overshoots 5 degrees even at
      // slow speeds
      m_angle((percentSpeed > 0) ? (angle - 5_deg) : (-angle + 5_deg)) {
  AddRequirements(m_drivebase);
}

// Called when the command is initially scheduled.
void RotateAtAngle::Initialize() {
  m_multiplier = 1;

  m_drivebase->SetBrakingMode(true);
  if (m_angle >= 0_deg)
    m_drivebase->TankDrive(-m_percentSpeed, m_percentSpeed);
  else
    m_drivebase->TankDrive(m_percentSpeed, -m_percentSpeed);
  m_startAngle = m_drivebase->GetAngle();
}

// Called repeatedly when this Command is scheduled to run
void RotateAtAngle::Execute() {
  m_drivebase->SetBrakingMode(true);
  units::degree_t currentPosition = m_drivebase->GetAngle();

  units::degree_t degreesLeft = (m_startAngle + m_angle) - currentPosition;
  units::degree_t degreesLeftWhenSlowDown = m_percentSpeed * 150_deg;

  if (m_angle >= 0_deg) {
    if (degreesLeft < degreesLeftWhenSlowDown &&
        m_percentSpeed * m_multiplier > 0.27) {
      m_multiplier *= .95;
      m_multiplier = (m_multiplier * m_percentSpeed > 0.27
                          ? m_multiplier * m_percentSpeed
                          : 0.27);  // speed must be >= 0.27
    }
    m_drivebase->TankDrive(-m_percentSpeed * m_multiplier,
                           m_percentSpeed * m_multiplier);
  }

  else {
    if (-degreesLeft < degreesLeftWhenSlowDown &&
        m_percentSpeed * m_multiplier > 0.27) {
      m_multiplier *= .95;
      m_multiplier =
          (m_multiplier * m_percentSpeed > 0.27 ? m_multiplier * m_percentSpeed
                                                : 0.27);
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
  units::degree_t currentAngle = m_drivebase->GetAngle();
  if (m_angle >= 0_deg)
    return (currentAngle >= m_angle + m_startAngle);
  else
    return (currentAngle <= m_angle + m_startAngle);
}
