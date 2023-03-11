// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExtendIntake.h"

#include "Constants.h"

ExtendIntake::ExtendIntake(IntakeDeployment* IntakeDeployment, double speed)
    : m_intakeDeployment(IntakeDeployment), intakeSpeed(std::abs(speed)) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements(m_intakeDeployment);
}

// Called when the command is initially scheduled.
void ExtendIntake::Initialize() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
  m_intakeDeployment->EnableBraking(true);
}

// Called repeatedly when this Command is scheduled to run
void ExtendIntake::Execute() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
}

// Called once the command ends or is interrupted.
void ExtendIntake::End(bool interrupted) {
  m_intakeDeployment->Stop();
  m_intakeDeployment->EnableBraking(true);
}

// Returns true when the command should end.
bool ExtendIntake::IsFinished() {
#ifdef ENABLE_INTAKE_LIMIT_SWITCH
  return m_intakeDeployment->IsIntakeDeployed(true);
#endif
  // return m_intakeDeployment->GetLeftVelocity() < Intake::STOP_VELOCITY;
  return false;
}
