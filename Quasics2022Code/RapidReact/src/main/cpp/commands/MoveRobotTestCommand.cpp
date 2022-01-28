// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveRobotTestCommand.h"

MoveRobotTestCommand::MoveRobotTestCommand(Drivebase* drivebase, double motorPower) : m_drivebase(drivebase), motorPower(0.2) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void MoveRobotTestCommand::Initialize() {
  m_drivebase->SetMotorPower(motorPower, motorPower);
}
  
// Called repeatedly when this Command is scheduled to run
void MoveRobotTestCommand::Execute() {
  m_drivebase->SetMotorPower(motorPower, motorPower);
}

// Called once the command ends or is interrupted.
void MoveRobotTestCommand::End(bool interrupted) {
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool MoveRobotTestCommand::IsFinished() {
  units::meter_t rotation = m_drivebase->GetLeftEncoders();
  if (rotation >= units::meter_t(1)){
    return true;
  }
  return false;
  }

