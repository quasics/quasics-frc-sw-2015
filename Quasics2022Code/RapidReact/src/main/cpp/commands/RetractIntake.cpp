// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RetractIntake.h"

RetractIntake::RetractIntake(IntakeDeployment* intake, double speed)
    : m_intakeDeployment(intake), intakeSpeed(speed) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements(m_intakeDeployment);
}

// Called when the command is initially scheduled.
void RetractIntake::Initialize() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
}

// Called repeatedly when this Command is scheduled to run
void RetractIntake::Execute() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
}

// Called once the command ends or is interrupted.
void RetractIntake::End(bool interrupted) {
  m_intakeDeployment->SetMotorSpeed(0);
}

// Returns true when the command should end.
bool RetractIntake::IsFinished() {
  return false;
}
