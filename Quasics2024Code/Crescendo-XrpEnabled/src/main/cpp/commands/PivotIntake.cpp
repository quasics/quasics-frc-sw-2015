// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PivotIntake.h"

#include <iostream>

PivotIntake::PivotIntake(IntakeDeployment &IntakeDeployment, double speed,
                         bool extend)
    : m_intakeDeployment(IntakeDeployment),
      m_intakeDeploymentSpeed(extend ? std::abs(speed) : -std::abs(speed)),
      m_extending(extend) {
  AddRequirements(&m_intakeDeployment);
  SetName("PivotIntake");
}

// Called when the command is initially scheduled.
void PivotIntake::Initialize() {
  m_intakeSlewRateLimiter.Reset(0);
  double speed = m_intakeSlewRateLimiter.Calculate(m_intakeDeploymentSpeed);
  std::cout << "Speed sending: " << speed << std::endl;
  m_intakeDeployment.SetMotorSpeed(speed);
  m_intakeDeployment.EnableBraking(false);
}

// Called repeatedly when this Command is scheduled to run
void PivotIntake::Execute() {
  double speed = m_intakeSlewRateLimiter.Calculate(m_intakeDeploymentSpeed);
  std::cout << "Speed sending: " << speed << std::endl;
  m_intakeDeployment.SetMotorSpeed(speed);
}

// Called once the command ends or is interrupted.
void PivotIntake::End(bool interrupted) {
  m_intakeDeployment.Stop();
  m_intakeDeployment.EnableBraking(false);
}
