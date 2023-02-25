// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveFloorEjection.h"

MoveFloorEjection::MoveFloorEjection(FloorEjection* floorEjection, double power)
    : m_floorEjection(floorEjection), m_power(power) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(floorEjection);
}

// Called when the command is initially scheduled.
void MoveFloorEjection::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MoveFloorEjection::Execute() {}

// Called once the command ends or is interrupted.
void MoveFloorEjection::End(bool interrupted) {}

// Returns true when the command should end.
bool MoveFloorEjection::IsFinished() { return false; }
