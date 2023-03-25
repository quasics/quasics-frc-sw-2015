// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/DriveAtPowerForMeters.h"

#include <iostream>

DriveAtPowerForMeters::DriveAtPowerForMeters(Drivebase* drivebase,
                                             double motorPower,
                                             units::meter_t distance)
    : m_drivebase(drivebase),
      m_motorPower(motorPower),
      m_distance(distance > 0_m ? distance - 1_in : distance + 1_in) {
  if (m_motorPower < 0) {
    if (m_distance < 0_m) {
      m_motorPower = -m_motorPower;
    } else {
      m_motorPower = -m_motorPower;
      m_distance = -m_distance;
    }
  }
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void DriveAtPowerForMeters::Initialize() {
  m_multiplier = 1;
  m_leftStartingPosition = m_drivebase->GetLeftDistance();
  m_rightStartingPosition = m_drivebase->GetRightDistance();

  if (m_distance > 0_m) {
    m_drivebase->TankDrive(m_motorPower, m_motorPower);

  }

  else {
    m_drivebase->TankDrive(-m_motorPower, -m_motorPower);
  }
}

// Called repeatedly when this Command is scheduled to run
void DriveAtPowerForMeters::Execute() {
  const double minimumSpeed = 0.4;
  const double scalingFactor = 0.8;

  // use left position: both left and right should be the same
  units::meter_t positionLeft = m_drivebase->GetLeftDistance();
  units::meter_t distanceLeft =
      (m_leftStartingPosition + m_distance) - positionLeft;
  units::meter_t distanceLeftWhenSlowDown = m_motorPower * 1_m;

  if (m_distance > 0_m) {
    if (distanceLeft <= distanceLeftWhenSlowDown &&
        m_motorPower * m_multiplier > minimumSpeed) {
      m_multiplier *= scalingFactor;
      m_multiplier = (m_multiplier * m_motorPower > minimumSpeed
                          ? m_multiplier
                          : minimumSpeed / m_motorPower);
    }
    m_drivebase->TankDrive(m_motorPower * m_multiplier,
                           m_motorPower * m_multiplier);

  } else {
    if (distanceLeft >= -distanceLeftWhenSlowDown &&
        m_motorPower * m_multiplier > minimumSpeed) {
      m_multiplier *= scalingFactor;
      m_multiplier = (m_multiplier * m_motorPower > minimumSpeed
                          ? m_multiplier
                          : minimumSpeed / m_motorPower);
    }
    m_drivebase->TankDrive(-m_motorPower * m_multiplier,
                           -m_motorPower * m_multiplier);
  }
}

// Called once the command ends or is interrupted.
void DriveAtPowerForMeters::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool DriveAtPowerForMeters::IsFinished() {
  // use left position: both left and right should be the same
  units::meter_t positionLeft = m_drivebase->GetLeftDistance();

  if (m_distance > 0_m) {
    return (positionLeft >= (m_leftStartingPosition + m_distance));
  } else {
    return (positionLeft <= (m_leftStartingPosition + m_distance));
  }
}
