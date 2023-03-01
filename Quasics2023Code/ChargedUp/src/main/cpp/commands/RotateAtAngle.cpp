// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateAtAngle.h"

#include <units/angle.h>

#include <iostream>

RotateAtAngle::RotateAtAngle(Drivebase* drivebase, double percentSpeed,
                             units::degree_t angle)
    : m_drivebase(drivebase), m_percentSpeed(percentSpeed), m_angle(angle) {
  AddRequirements(m_drivebase);
}

// Called when the command is initially scheduled.
void RotateAtAngle::Initialize() {
  // CODE_REVIEW: This is making undocumented assumptions about the signs for
  // speed and angle.  Please either add some comments in the header to clarify
  // how they work/should be used, or normalize them above, or something.  (For
  // example, what if the user passes in -90 degrees, and -0.5 for speed?)
  if (m_percentSpeed < 0) {
    if (m_angle < 0_deg) {
      m_percentSpeed = -m_percentSpeed;
    } else {
      m_angle = -m_angle;
      m_percentSpeed = -m_percentSpeed;
    }
  }

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
  const double minimumSpeed = 0.30;  // speed must be >= 0.30
  const double scalingFactor = 0.98;

  const units::degree_t currentPosition = m_drivebase->GetAngle();

  const units::degree_t degreesLeft =
      (m_startAngle + m_angle) - currentPosition;
  std::cerr << degreesLeft.value() << std::endl;
  const units::degree_t degreesLeftWhenSlowDown = m_angle / 4;

  if (m_angle >= 0_deg) {
    if (degreesLeft < degreesLeftWhenSlowDown &&
        m_percentSpeed * m_multiplier > minimumSpeed) {
      m_multiplier *= scalingFactor;
      m_multiplier = (m_multiplier * m_percentSpeed > minimumSpeed
                          ? m_multiplier * m_percentSpeed
                          : minimumSpeed);
    }
    m_drivebase->TankDrive(-m_percentSpeed * m_multiplier,
                           m_percentSpeed * m_multiplier);
  } else {
    if (-degreesLeft < degreesLeftWhenSlowDown &&
        m_percentSpeed * m_multiplier > minimumSpeed) {
      m_multiplier *= scalingFactor;
      m_multiplier = (m_multiplier * m_percentSpeed > minimumSpeed
                          ? m_multiplier * m_percentSpeed
                          : minimumSpeed);
    }
    m_drivebase->TankDrive(m_percentSpeed * m_multiplier,
                           -m_percentSpeed * m_multiplier);
  }
}

// Called once the command ends or is interrupted.
void RotateAtAngle::End(bool interrupted) {
  units::degree_t currentPosition = m_drivebase->GetAngle();
  units::degree_t degreesLeft = (m_startAngle + m_angle) - currentPosition;
  std::cerr << "Final value: " << degreesLeft.value() << std::endl;
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
