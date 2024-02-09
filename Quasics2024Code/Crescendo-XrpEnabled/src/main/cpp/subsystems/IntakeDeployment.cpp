// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeDeployment.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

IntakeDeployment::IntakeDeployment() {
  SetName("IntakeDeployment");
}

// This method will be called once per scheduler run
void IntakeDeployment::SetMotorSpeed(double percentSpeed) {
  m_intakeDeployment.Set(percentSpeed);
}

void IntakeDeployment::Stop() {
  m_intakeDeployment.StopMotor();
}

void IntakeDeployment::ResetEncoders() {
  m_DeploymentEncoder.SetPosition(0);
}

double IntakeDeployment::GetPosition() {
  return m_DeploymentEncoder.GetPosition();
}

double IntakeDeployment::GetVelocity() {
  return m_DeploymentEncoder.GetVelocity();
}

void IntakeDeployment::EnableBraking(bool value) {
  rev::CANSparkMax::IdleMode mode;
  if (value) {
    mode = rev::CANSparkMax::IdleMode::kBrake;
  } else {
    mode = rev::CANSparkMax::IdleMode::kCoast;
  }
  m_intakeDeployment.SetIdleMode(mode);
}

bool IntakeDeployment::IsIntakeDeployed() {
  return !m_ExtendIntakeLimitSwitch.Get();
}

bool IntakeDeployment::IsIntakeRetracted() {
  return !m_RetractIntakeLimitSwitch.Get();
}

double IntakeDeployment::GetRevolutions() {
  return m_DeploymentEncoder.GetPosition() / 42;
}

bool IntakeDeployment::ExtendedByRevolutions() {
  // ARBITRARY NUMBER PLEASE SWITCH
  if (GetRevolutions() > 100) {
    return true;
  }
  return false;
}

bool IntakeDeployment::RetractedByRevolutions() {
  // ARBITRARY NUMBER PLEASE SWITCH
  if (GetRevolutions() < 0) {
    return true;
  }
  return false;
}

void IntakeDeployment::Periodic() {
  /* frc::SmartDashboard::PutString(
       "Retract limit switch",
       m_RetractIntakeLimitSwitch.Get() ? "open" : "closed");

   frc::SmartDashboard::PutString(
       "Extend limit switch",
       m_ExtendIntakeLimitSwitch.Get() ? "open" : "closed");*/

  frc::SmartDashboard::PutNumber("Deployment Revolutions", GetRevolutions());
}
