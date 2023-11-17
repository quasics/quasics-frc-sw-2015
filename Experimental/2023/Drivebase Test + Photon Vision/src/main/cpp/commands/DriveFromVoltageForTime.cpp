// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveFromVoltageForTime.h"

DriveFromVoltageForTime::DriveFromVoltageForTime(Drivebase* drivebase, units::volt_t leftVoltage, units::volt_t rightVoltage, units::second_t time) : 
  m_drivebase(drivebase), m_leftVoltage(leftVoltage), m_rightVoltage(rightVoltage), m_time(time) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements(m_drivebase);
}

// Called when the command is initially scheduled.
void DriveFromVoltageForTime::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_drivebase->TankDriveVolts(m_leftVoltage, m_rightVoltage);
}

// Called repeatedly when this Command is scheduled to run
void DriveFromVoltageForTime::Execute() {
  m_drivebase->TankDriveVolts(m_leftVoltage, m_rightVoltage);
}

// Called once the command ends or is interrupted.
void DriveFromVoltageForTime::End(bool interrupted) {
  m_drivebase->TankDriveVolts(0_V, 0_V);
}

// Returns true when the command should end.
bool DriveFromVoltageForTime::IsFinished() {
  if (m_stopWatch.HasElapsed(m_time)) {
    return true;
  }
  return false;
}
