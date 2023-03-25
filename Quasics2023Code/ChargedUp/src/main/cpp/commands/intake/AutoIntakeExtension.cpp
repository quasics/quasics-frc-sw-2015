// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/AutoIntakeExtension.h"

AutoIntakeExtension::AutoIntakeExtension(IntakeDeployment* intakeDeployment,
                                         double deploymentSpeed)
    : m_intakeDeployment(intakeDeployment),
      m_deploymentSpeed(std::abs(deploymentSpeed)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intakeDeployment);
}

// Called when the command is initially scheduled.
void AutoIntakeExtension::Initialize() {
  m_intakeDeployment->SetMotorSpeed(m_deploymentSpeed);
}

// Called repeatedly when this Command is scheduled to run
void AutoIntakeExtension::Execute() {
  m_intakeDeployment->SetMotorSpeed(m_deploymentSpeed);
}

// Called once the command ends or is interrupted.
void AutoIntakeExtension::End(bool interrupted) {
  m_intakeDeployment->Stop();
}

// Returns true when the command should end.
bool AutoIntakeExtension::IsFinished() {
  if (m_intakeDeployment->IsIntakeDeployed(
          IntakeDeployment::LimitSwitch::Extended)) {
    return true;
  }
  return false;
}
