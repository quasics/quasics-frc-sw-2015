// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveRobotTestCommand.h"

MoveRobotTestCommand::MoveRobotTestCommand(Drivebase* drivebase, double motorPower) : m_drivebase(drivebase), m_motorPower(motorPower), m_leftStartingPosition(0), m_rightStartingPosition(0) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void MoveRobotTestCommand::Initialize() {
  m_leftStartingPosition = m_drivebase -> GetLeftEncoders();
  m_rightStartingPosition = m_drivebase-> GetRightEncoders();
  m_drivebase->SetMotorPower(m_motorPower, m_motorPower);
}
  
// Called repeatedly when this Command is scheduled to run
void MoveRobotTestCommand::Execute() {
  m_drivebase->SetMotorPower(m_motorPower, m_motorPower);
}

// Called once the command ends or is interrupted.
void MoveRobotTestCommand::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool MoveRobotTestCommand::IsFinished() {
  units::meter_t rotationLeft = m_drivebase->GetLeftEncoders();
  units::meter_t rotationRight = m_drivebase->GetRightEncoders();
  if (rotationLeft >= (m_leftStartingPosition + units::meter_t(10.71)) or (rotationRight >= (m_rightStartingPosition + units::meter_t(10.71)))){
    return true;
  }
  return false;
  }

