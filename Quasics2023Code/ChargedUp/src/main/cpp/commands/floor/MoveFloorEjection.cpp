// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/floor/MoveFloorEjection.h"

#include "Constants.h"

MoveFloorEjection::MoveFloorEjection(FloorEjection* floorEjection, double power)
    : m_floorEjection(floorEjection), m_power(power) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(floorEjection);
  SetName("MoveFloorEjection");
}

// Called when the command is initially scheduled.
void MoveFloorEjection::Initialize() {
  m_floorEjection->SetFloorEjectionPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void MoveFloorEjection::Execute() {
  m_floorEjection->SetFloorEjectionPower(m_power);
}

// Called once the command ends or is interrupted.
void MoveFloorEjection::End(bool interrupted) {
  m_floorEjection->SetFloorEjectionPower(0);
}

// Returns true when the command should end.
bool MoveFloorEjection::IsFinished() {
#ifdef ENABLE_FLOOR_EJECTION_ENCODER
  return m_floorEjection->GetVelocity() < Floor::STOP_VELOCITY;
#endif
  return false;
}
