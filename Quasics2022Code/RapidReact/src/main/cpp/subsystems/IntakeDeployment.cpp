// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeDeployment.h"

#include <frc/smartdashboard/SmartDashboard.h>

IntakeDeployment::IntakeDeployment() {
}

// This method will be called once per scheduler run
void IntakeDeployment::SetMotorSpeed(double percentSpeed) {
  // dont know about this line maybe worth deleting
  m_IntakeDeploymentMotor.SetNeutralMode(
      ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_IntakeDeploymentMotor.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput,
      percentSpeed);
}

void IntakeDeployment::SetBrakingMode(bool brake) {
  if (brake) {
    m_IntakeDeploymentMotor.SetNeutralMode(
        ctre::phoenix::motorcontrol::NeutralMode::Brake);
  } else {
    m_IntakeDeploymentMotor.SetNeutralMode(
        ctre::phoenix::motorcontrol::NeutralMode::Coast);
  }
}

bool IntakeDeployment::IsIntakeDeployed() {
  return !intakeLimitSwitch.Get();
}

void IntakeDeployment::Periodic() {
  frc::SmartDashboard::PutString("Limit switch",
                                 intakeLimitSwitch.Get() ? "open" : "closed");
}
