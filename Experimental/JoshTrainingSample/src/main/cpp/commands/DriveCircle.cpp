// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCircle.h"

DriveCircle::DriveCircle(Drivetrain* drivetrain, units::meter_t distance, double speedAsPercent) 
  : m_drivetrain(drivetrain), m_distance(distance), m_speedAsPercent(speedAsPercent) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrain});
}

// Called when the command is initially scheduled.
void DriveCircle::Initialize() {
  m_drivetrain->ResetGyro();
  m_drivetrain->ResetEncoders();
  m_drivetrain->TankDrive(m_speedAsPercent, (m_speedAsPercent * 0.75));
}
/* (((speedAsPercent - 0.1397) * 3.1415) / (speedAsPercent * 3.1415)) */
// Called repeatedly when this Command is scheduled to run
void DriveCircle::Execute() {}

// Called once the command ends or is interrupted.
void DriveCircle::End(bool interrupted) {
  m_drivetrain->TankDrive(0,0);
}

// Returns true when the command should end.
bool DriveCircle::IsFinished() {
  if(m_drivetrain->GetGyroAngleZ() >= 360) {
    return true
  }
  return false;
}
