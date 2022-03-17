// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateAtSpeedForDegrees.h"

#include "subsystems/Drivebase.h"

// RotateAtSpeedForDegrees::RotateAtSpeedForDegrees() {
// }
RotateAtSpeedForDegrees::RotateAtSpeedForDegrees(Drivebase* drivebase,
                                                 units::degree_t angle)
    : m_drivebase(drivebase), m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drivebase);
}

// Called when the command is initially scheduled.
void RotateAtSpeedForDegrees::Initialize() {
  // units::degree_t position = m_drivebase->GetAngle();
  m_drivebase->SetLeftMotorPower(-0.6);
  m_drivebase->SetRightMotorPower(0.6);
}

// Called repeatedly when this Command is scheduled to run
void RotateAtSpeedForDegrees::Execute() {
}

// Called once the command ends or is interrupted.
void RotateAtSpeedForDegrees::End(bool interrupted) {
}

// Returns true when the command should end.
bool RotateAtSpeedForDegrees::IsFinished() {
  return false;
}
