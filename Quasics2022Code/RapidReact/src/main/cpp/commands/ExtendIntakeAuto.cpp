// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExtendIntakeAuto.h"

ExtendIntakeAuto::ExtendIntakeAuto(IntakeDeployment* IntakeDeployment,
                                   double speed)
    : m_intakeDeployment(IntakeDeployment), intakeSpeed(speed) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements(m_intakeDeployment);
}

// Called when the command is initially scheduled.
void ExtendIntakeAuto::Initialize() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
}

// Called repeatedly when this Command is scheduled to run
void ExtendIntakeAuto::Execute() {
  double adjusted = intakeSpeed * multiplier;
  if (adjusted > 0.2) {
    m_intakeDeployment->SetMotorSpeed(adjusted);
    multiplier *= 0.999;
  }
  m_intakeDeployment->SetMotorSpeed(adjusted);
}

// Called once the command ends or is interrupted.
void ExtendIntakeAuto::End(bool interrupted) {
  m_intakeDeployment->SetMotorSpeed(0);
}

// Returns true when the command should end.
bool ExtendIntakeAuto::IsFinished() {
  return m_intakeDeployment->IsIntakeDeployed();
}
