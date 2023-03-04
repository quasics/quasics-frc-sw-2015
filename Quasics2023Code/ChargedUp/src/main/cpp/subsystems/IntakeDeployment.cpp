// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeDeployment.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

IntakeDeployment::IntakeDeployment() {
#ifdef ENABLE_INTAKE_DEPLOYMENT_MOTORS
  std::cerr << "Intake deployment is enabled\n";
#else
  std::cerr << "Intake deployment is NOT enabled\n";
#endif
}

// This method will be called once per scheduler run
void IntakeDeployment::SetMotorSpeed(double percentSpeed) {
#ifdef ENABLE_INTAKE_DEPLOYMENT_MOTORS
  m_intakeDeployment.Set(percentSpeed);
#endif
}

void IntakeDeployment::Stop() {
#ifdef ENABLE_INTAKE_DEPLOYMENT_MOTORS
  m_intakeDeployment.StopMotor();
#endif
}

void IntakeDeployment::ResetEncoders() {
#ifdef ENABLE_INTAKE_DEPLOYMENT_MOTORS
  m_leftDeploymentEncoder.SetPosition(0);
  m_rightDeploymentEncoder.SetPosition(0);
#endif
}

double IntakeDeployment::GetLeftPosition() {
#ifdef ENABLE_INTAKE_DEPLOYMENT_MOTORS
  return m_leftDeploymentEncoder.GetPosition();  // rotations
#endif
}

double IntakeDeployment::GetLeftVelocity() {
#ifdef ENABLE_INTAKE_DEPLOYMENT_MOTORS
  return m_leftDeploymentEncoder.GetVelocity();  // RPM
#endif
}

double IntakeDeployment::GetRightPosition() {
#ifdef ENABLE_INTAKE_DEPLOYMENT_MOTORS
  return m_rightDeploymentEncoder.GetPosition();
#endif
}

double IntakeDeployment::GetRightVelocity() {
#ifdef ENABLE_INTAKE_DEPLOYMENT_MOTORS
  return m_rightDeploymentEncoder.GetVelocity();
#endif
}

void IntakeDeployment::EnableBraking(bool value) {
#ifdef ENABLE_INTAKE_DEPLOYMENT_MOTORS
  rev::CANSparkMax::IdleMode mode;
  if (value) {
    mode = rev::CANSparkMax::IdleMode::kBrake;
  } else {
    mode = rev::CANSparkMax::IdleMode::kCoast;
  }
  m_leftDeploymentMotor.SetIdleMode(mode);
  m_rightDeploymentMotor.SetIdleMode(mode);
#else
//(void)mode;
#endif
}

bool IntakeDeployment::IsIntakeDeployed() {
  // possible function that migbht need to be implemented
  return false;
}
void IntakeDeployment::Periodic() {
  frc::SmartDashboard::PutNumber("Roller Position",
                                 m_leftDeploymentEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Roller Velocity",
                                 m_rightDeploymentEncoder.GetVelocity());
}
