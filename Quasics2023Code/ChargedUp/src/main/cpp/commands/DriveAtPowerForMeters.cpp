// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveAtPowerForMeters.h"
#include <iostream>

DriveAtPowerForMeters::DriveAtPowerForMeters(Drivebase* drivebase, double motorPower, units::meter_t distance) 
  : m_drivebase(drivebase), m_motorPower(motorPower), m_distance(distance) {
    AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void DriveAtPowerForMeters::Initialize() {
  // m_drivebase->ResetEncoders();
  m_leftStartingPosition = m_drivebase->GetLeftDistance();
  m_rightStartingPosition = m_drivebase->GetRightDistance();
  //std::cerr << "left: " << m_leftStartingPosition.value() << ", right: " << m_rightStartingPosition.value() << std::endl;
  m_drivebase->TankDrive(m_motorPower, m_motorPower);
}


// Called repeatedly when this Command is scheduled to run
void DriveAtPowerForMeters::Execute() {
  double multiplier;
  units::meter_t positionLeft = m_drivebase->GetLeftDistance();
  units::meter_t positionRight = m_drivebase->GetRightDistance();
  
  if (m_distance > 0_m) {
    if ((positionLeft >= (m_leftStartingPosition + m_distance) - 0.2) || 
        (positionRight >= (m_rightStartingPosition + m_distance) - 0.2 )) {
      multiplier = 0.2;
    }
  }

  else {
    if ((positionLeft >= (m_leftStartingPosition + m_distance) - 0.2) ||
        (positionRight >= (m_rightStartingPosition + m_distance) - 0.2 )) {
      multiplier = 0.2;
    }
  }

  m_drivebase->TankDrive(m_motorPower * multiplier, m_motorPower * multiplier);
}

// Called once the command ends or is interrupted.
void DriveAtPowerForMeters::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
  //std::cerr << "Start Left: " << m_leftStartingPosition.value() << ", Start Right: "
  // << m_rightStartingPosition.value() << ", Distance: " << m_distance.value() << std::endl;
}

// Returns true when the command should end.
bool DriveAtPowerForMeters::IsFinished() {
  units::meter_t positionLeft = m_drivebase->GetLeftDistance();
  units::meter_t positionRight = m_drivebase->GetRightDistance();
  // since driving straight, positionLeft and positionRight should be very similar
  
  //std::cerr << "Left: " << positionLeft.value() << ", Right: " << positionRight.value() << std::endl;

  if (m_distance > 0_m) {
    if ((positionLeft >= (m_leftStartingPosition + m_distance)) ||
        (positionRight >= (m_rightStartingPosition + m_distance))) {
      return true;
    } else {
      return false;
    }
  } else {
    if ((positionLeft <= (m_leftStartingPosition + m_distance)) ||
        (positionRight <= (m_rightStartingPosition + m_distance))) {
      return true;
    } else {
      return false;
    }
  }
}
