// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeClamp.h"

IntakeClamp::IntakeClamp() = default;

// This method will be called once per scheduler run
void IntakeClamp::Periodic() {}

void IntakeClamp::SetIntakeClampSpeed(double percentSpeed) {
  m_intakeClamp.Set(percentSpeed);
}

void IntakeClamp::Stop() { m_intakeClamp.StopMotor(); }

void IntakeClamp::EnableBraking(bool value) {
  rev::CANSparkMax::IdleMode mode;
  if (value) {
    mode = rev::CANSparkMax::IdleMode::kBrake;
  } else {
    mode = rev::CANSparkMax::IdleMode::kCoast;
  }
  m_intakeClamp.SetIdleMode(mode);
}

bool IntakeClamp::IsIntakeClampDeployed() {
  // possible function that might need to be implemeneted to prevent the intake
  // from overextending
  return false;
}
