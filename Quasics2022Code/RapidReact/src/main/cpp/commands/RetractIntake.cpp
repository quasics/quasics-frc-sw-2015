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
  m_intakeDeployment->SetBrakingMode(true);
}

// Called repeatedly when this Command is scheduled to run
void RetractIntake::Execute() {
  if (intakeSpeed * multiplier < -0.1) {
    multiplier = multiplier * 0.99;
  }
  m_intakeDeployment->SetMotorSpeed(intakeSpeed * multiplier);
  // m_intakeDeployment->SetMotorSpeed(intakeSpeed);
  // m_intakeDeployment->SetBrakingMode(true);
}

// Called once the command ends or is interrupted.
void RetractIntake::End(bool interrupted) {
  // maybe delete these and then it will be overided whenveer they extend intake
  m_intakeDeployment->SetMotorSpeed(0);
  m_intakeDeployment->SetBrakingMode(true);
}

// Returns true when the command should end.
bool RetractIntake::IsFinished() {
  return false;
}
