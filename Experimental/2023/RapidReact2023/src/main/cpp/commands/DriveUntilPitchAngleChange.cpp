// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveUntilPitchAngleChange.h"

DriveUntilPitchAngleChange::DriveUntilPitchAngleChange(Drivebase* drivebase, double power) : m_drivebase(drivebase), m_power(power) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void DriveUntilPitchAngleChange::Initialize() {
  m_drivebase->SetMotorPower(m_power, m_power);
}

// Called repeatedly when this Command is scheduled to run
void DriveUntilPitchAngleChange::Execute() {
  m_drivebase->SetMotorPower(m_power, m_power);
}

// Called once the command ends or is interrupted.
void DriveUntilPitchAngleChange::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool DriveUntilPitchAngleChange::IsFinished() {
  double currentAngle = m_drivebase->GetPitch();
  if (currentAngle > 10 || currentAngle < -10){
    return true;
  }
  return false;
}
