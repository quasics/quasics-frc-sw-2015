// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootTheGamePiece.h"

ShootTheGamePiece::ShootTheGamePiece(FloorEjection* floorEjection, double angle,
                                     double power)
    : m_floorEjection(floorEjection), m_angle(angle), m_power(power) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(floorEjection);
}

// Called when the command is initially scheduled.
void ShootTheGamePiece::Initialize() {
  m_floorEjection->SetFloorEjectionPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void ShootTheGamePiece::Execute() {
  m_floorEjection->SetFloorEjectionPower(m_power);
}

// Called once the command ends or is interrupted.
void ShootTheGamePiece::End(bool interrupted) {
  m_floorEjection->Stop();
}

// Returns true when the command should end.
bool ShootTheGamePiece::IsFinished() {
  double floorPosition = m_floorEjection->GetPosition();
  if (floorPosition >= m_angle) {
    return true;
  }
  return false;
}
