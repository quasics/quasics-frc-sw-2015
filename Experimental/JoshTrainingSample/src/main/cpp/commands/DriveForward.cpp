// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveForward.h"

DriveForward::DriveForward(Drivetrain* drivetrain, units::meter_t distance, double speedAsPercent)
  : m_drivetrain(drivetrain), m_distance(distance), m_speedAsPercent(speedAsPercent) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrain});
}

// Called when the command is initially scheduled.
void DriveForward::Initialize() {
  m_drivetrain->ResetEncoders();
  m_drivetrain->ArcadeDrive(m_speedAsPercent, 0);
}

// Called repeatedly when this Command is scheduled to run
void DriveForward::Execute() {}

// Called once the command ends or is interrupted.
void DriveForward::End(bool interrupted) {
  m_drivetrain->ArcadeDrive(0, 0);
}

// Returns true when the command should end.
bool DriveForward::IsFinished() {
  if (m_drivetrain->GetAverageDistance() >= m_distance) {
    return true;
  }
  
  return false;
}
