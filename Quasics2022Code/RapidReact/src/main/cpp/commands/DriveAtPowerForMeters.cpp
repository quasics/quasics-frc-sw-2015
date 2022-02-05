// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveAtPowerForMeters.h"

DriveAtPowerForMeters::DriveAtPowerForMeters(Drivebase* drivebase, double motorPower, double distance) : m_drivebase(drivebase), m_motorPower(motorPower), m_distance(distance) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void DriveAtPowerForMeters::Initialize() {
  m_leftStartingPosition = m_drivebase -> GetLeftDistance();
  m_rightStartingPosition = m_drivebase -> GetRightDistance();
  m_drivebase -> SetMotorPower(m_motorPower, m_motorPower);
}

// Called repeatedly when this Command is scheduled to run
void DriveAtPowerForMeters::Execute() {}

// Called once the command ends or is interrupted.
void DriveAtPowerForMeters::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveAtPowerForMeters::IsFinished() {
  return false;
}
