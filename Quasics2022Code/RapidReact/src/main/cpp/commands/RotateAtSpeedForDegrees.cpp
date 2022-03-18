// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateAtSpeedForDegrees.h"

#include "subsystems/Drivebase.h"

// RotateAtSpeedForDegrees::RotateAtSpeedForDegrees() {
// }
RotateAtSpeedForDegrees::RotateAtSpeedForDegrees(Drivebase* drivebase,
                                                 double speed,
                                                 units::degree_t angle)
    : m_drivebase(drivebase), m_speed(speed), m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drivebase);
}

// Called when the command is initially scheduled.
void RotateAtSpeedForDegrees::Initialize() {
  startingposition = m_drivebase->GetAngle();
  m_drivebase->SetBrakingMode(true);
  m_drivebase->SetMotorPower(-1 * m_speed, m_speed);
  //     m_drivebase->SetLeftMotorPower(-1 * m_speed);
  // m_drivebase->SetRightMotorPower(m_speed);
}

// Called repeatedly when this Command is scheduled to run
void RotateAtSpeedForDegrees::Execute() {
  m_drivebase->SetBrakingMode(true);
  units::degree_t currentPosition = m_drivebase->GetAngle();
  if (currentPosition > ((startingposition + m_angle) * 0.5) &&
      (m_speed * multiplier > 0.26)) {
    multiplier = multiplier * 0.99;
  }
  m_drivebase->SetMotorPower(-1 * m_speed * multiplier, m_speed * multiplier);
  // m_drivebase->SetLeftMotorPower(-1 * m_speed);
  // m_drivebase->SetRightMotorPower(m_speed);
}

// Called once the command ends or is interrupted.
void RotateAtSpeedForDegrees::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->SetMotorPower(0, 0);
  // m_drivebase->SetLeftMotorPower(0);
  // m_drivebase->SetRightMotorPower(0);
}

// Returns true when the command should end.
bool RotateAtSpeedForDegrees::IsFinished() {
  units::degree_t currentPosition = m_drivebase->GetAngle();
  if (currentPosition >= (startingposition + m_angle)) {
    return true;
  }
  return false;
}
