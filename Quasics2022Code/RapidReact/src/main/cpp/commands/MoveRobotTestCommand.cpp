// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveRobotTestCommand.h"

MoveRobotTestCommand::MoveRobotTestCommand(Drivebase* drivebase,
                                           double motorPower)
    : m_drivebase(drivebase),
      m_motorPower(motorPower),
      m_leftStartingPosition(0),
      m_rightStartingPosition(0) {
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void MoveRobotTestCommand::Initialize() {
  m_leftStartingPosition = m_drivebase->GetLeftDistance();
  m_rightStartingPosition = m_drivebase->GetRightDistance();
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
  units::meter_t distanceleft = m_drivebase->GetLeftDistance();
  units::meter_t distanceright = m_drivebase->GetRightDistance();
  if (distanceleft >= (m_leftStartingPosition + units::meter_t(1)) or
      (distanceright >= (m_rightStartingPosition + units::meter_t(1)))) {
    return true;
  }
  return false;
}
