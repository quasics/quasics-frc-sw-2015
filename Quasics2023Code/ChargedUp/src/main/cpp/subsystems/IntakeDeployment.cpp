// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeDeployment.h"

#include <iostream>

IntakeDeployment::IntakeDeployment() {
#ifdef ENABLE_INTAKE_DEPLOYMENT
  std::cerr << "Intake deployment is enabled\n";
#else
  std::cerr << "Intake deployment is NOT enabled\n";
#endif
}

// This method will be called once per scheduler run
void IntakeDeployment::SetMotorSpeed(double percentSpeed) {
#ifdef ENABLE_INTAKE_DEPLOYMENT
  m_intakeDeployment.Set(percentSpeed);
#endif
}

void IntakeDeployment::Stop() {
#ifdef ENABLE_INTAKE_DEPLOYMENT
  m_intakeDeployment.StopMotor();
#endif
}

void IntakeDeployment::EnableBraking(bool value) {
#ifdef ENABLE_INTAKE_DEPLOYMENT
  rev::CANSparkMax::IdleMode mode;
  if (value) {
    mode = rev::CANSparkMax::IdleMode::kBrake;
  } else {
    mode = rev::CANSparkMax::IdleMode::kCoast;
  }
  m_leftDeploymentMotor.SetIdleMode(mode);
  m_rightDeploymentMotor.SetIdleMode(mode);
#endif
}

bool IntakeDeployment::IsIntakeDeployed() {
  // possible function that migbht need to be implemented
  return false;
}
void IntakeDeployment::Periodic() {
}
