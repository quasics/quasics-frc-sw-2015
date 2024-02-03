// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PivotIntake.h"

PivotIntake::PivotIntake(IntakeDeployment &IntakeDeployment, double speed,
                         bool extend)
    : m_intakeDeployment(IntakeDeployment),
      m_intakeDeploymentSpeed(extend ? std::abs(speed) : -std::abs(speed)),
      m_extending(extend) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&m_intakeDeployment);
  SetName("PivotIntake");
}

// Called when the command is initially scheduled.
void PivotIntake::Initialize() {
  m_intakeDeployment.SetMotorSpeed(m_intakeDeploymentSpeed);
  m_intakeDeployment.EnableBraking(true);
}

// Called repeatedly when this Command is scheduled to run
void PivotIntake::Execute() {
  m_intakeDeployment.SetMotorSpeed(m_intakeDeploymentSpeed);
}

// Called once the command ends or is interrupted.
void PivotIntake::End(bool interrupted) {
  m_intakeDeployment.Stop();
  m_intakeDeployment.EnableBraking(true);
}

// Returns true when the command should end.
bool PivotIntake::IsFinished() {
  return false;
}