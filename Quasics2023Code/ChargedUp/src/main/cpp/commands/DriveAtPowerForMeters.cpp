// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveAtPowerForMeters.h"

#include <iostream>

DriveAtPowerForMeters::DriveAtPowerForMeters(Drivebase* drivebase,
                                             double motorPower,
                                             units::meter_t distance)
    : m_drivebase(drivebase), m_motorPower(motorPower), m_distance(distance) {
  // m_distance can be positive (forwards) or negative (backwards), m_motorPower
  // will always be positive after this code even if user enters weird values.

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
  // m_drivebase->ResetEncoders();
  double newSpeed;
  m_leftStartingPosition = m_drivebase->GetLeftDistance();
  m_rightStartingPosition = m_drivebase->GetRightDistance();
  units::meter_t distanceLeft = m_distance;
  units::meter_t distanceLeftWhenSlowDown = 0.5_m;

  if (m_distance > 0_m) {
    if (distanceLeft <= distanceLeftWhenSlowDown) {
      newSpeed = 0.25;
    }
    m_drivebase->TankDrive(newSpeed, newSpeed);

  }

  else {
    if (distanceLeft >= -distanceLeftWhenSlowDown) {
      newSpeed = 0.25;
    }
    m_drivebase->TankDrive(-newSpeed, -newSpeed);
  }

  // std::cerr << "left: " << m_leftStartingPosition.value() << ", right: " <<
  // m_rightStartingPosition.value() << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void DriveAtPowerForMeters::Execute() {
  double newSpeed = m_motorPower;
  // use left position: both left and right should be the same
  units::meter_t positionLeft = m_drivebase->GetLeftDistance();
  units::meter_t distanceLeft =
      (m_leftStartingPosition + m_distance) - positionLeft;
  units::meter_t distanceLeftWhenSlowDown = 0.5_m;

  if (m_distance > 0_m) {
    if (distanceLeft <= distanceLeftWhenSlowDown) {
      newSpeed = 0.25;
    }
    m_drivebase->TankDrive(newSpeed, newSpeed);

  }

  else {
    if (distanceLeft >= -distanceLeftWhenSlowDown) {
      newSpeed = 0.25;
    }
    m_drivebase->TankDrive(-newSpeed, -newSpeed);
  }
}

// Called once the command ends or is interrupted.
void DriveAtPowerForMeters::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
  // std::cerr << "Start Left: " << m_leftStartingPosition.value() << ", Start
  // Right: "
  //  << m_rightStartingPosition.value() << ", Distance: " << m_distance.value()
  //  << std::endl;
}

// Returns true when the command should end.
bool DriveAtPowerForMeters::IsFinished() {
  // use left position: both left and right should be the same
  units::meter_t positionLeft = m_drivebase->GetLeftDistance();

  // std::cerr << "Left: " << positionLeft.value() << ", Right: " <<
  // positionRight.value() << std::endl;

  if (m_distance > 0_m) {
    return (positionLeft >= (m_leftStartingPosition + m_distance));
  } else {
    return (positionLeft <= (m_leftStartingPosition + m_distance));
  }
}
