// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeDeployment.h"

IntakeDeployment::IntakeDeployment() {
}

// This method will be called once per scheduler run
void IntakeDeployment::SetMotorSpeed(double percentSpeed) {
  m_IntakeDeploymentMotor.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput,
      percentSpeed);
}

void IntakeDeployment::Periodic() {
}
