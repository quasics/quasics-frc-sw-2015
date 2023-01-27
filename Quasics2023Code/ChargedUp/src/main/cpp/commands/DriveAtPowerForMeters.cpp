// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveAtPowerForMeters.h"

DriveAtPowerForMeters::DriveAtPowerForMeters(Drivebase* drivebase, double motorPower, units::meter_t distance) 
  : m_drivebase(drivebase), m_motorPower(motorPower), m_distance(distance) {
    AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void DriveAtPowerForMeters::Initialize() {
  m_leftStartingPosition = m_drivebase->GetLeftDistance();
  m_rightStartingPosition = m_drivebase->GetRightDistance();
  m_drivebase->TankDrive(m_motorPower, m_motorPower);
}


// Called repeatedly when this Command is scheduled to run
void DriveAtPowerForMeters::Execute() {
  m_drivebase->TankDrive(m_motorPower, m_motorPower);
}

// Called once the command ends or is interrupted.
void DriveAtPowerForMeters::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool DriveAtPowerForMeters::IsFinished() {
    units::meter_t positionLeft = m_drivebase->GetLeftDistance();
  units::meter_t positionRight = m_drivebase->GetRightDistance();
  if (m_distance > 0_m) {
    if ((positionLeft >= (m_leftStartingPosition + m_distance)) or
        (positionRight >= (m_rightStartingPosition + m_distance))) {
      return true;
    } else {
      return false;
    }
  } else {
    if ((positionLeft <= (m_leftStartingPosition + m_distance)) or
        (positionRight <= (m_rightStartingPosition + m_distance))) {
      return true;
    } else {
      return false;
    }
  }
}
