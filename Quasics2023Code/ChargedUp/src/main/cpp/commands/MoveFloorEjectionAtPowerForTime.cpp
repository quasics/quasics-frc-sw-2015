// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveFloorEjectionAtPowerForTime.h"

MoveFloorEjectionAtPowerForTime::MoveFloorEjectionAtPowerForTime(
    FloorEjection* floorEjection, double power, units::second_t time)
    : m_floorEjection(floorEjection), m_power(power), m_time(time) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(floorEjection);
}

// Called when the command is initially scheduled.
void MoveFloorEjectionAtPowerForTime::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_floorEjection->SetFloorEjectionPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void MoveFloorEjectionAtPowerForTime::Execute() {
  m_floorEjection->SetFloorEjectionPower(m_power);
}

// Called once the command ends or is interrupted.
void MoveFloorEjectionAtPowerForTime::End(bool interrupted) {
  m_floorEjection->Stop();
}

// Returns true when the command should end.
bool MoveFloorEjectionAtPowerForTime::IsFinished() {
  if (m_stopWatch.HasElapsed(m_time)) {
    return true;
  }
  return false;
}
