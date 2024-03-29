// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeClamp.h"

#include <iostream>

IntakeClamp::IntakeClamp() {
  SetName("IntakeClamp");

#ifdef ENABLE_CLAMP_MOTORS
  std::cerr << "Clamp motors are enabled\n";
#else
  std::cerr << "Clamp motors are NOT enabled\n";
#endif
}

// This method will be called once per scheduler run
void IntakeClamp::Periodic() {
}

void IntakeClamp::SetIntakeClampSpeed(double percentSpeed) {
#ifdef ENABLE_CLAMP_MOTORS
  m_intakeClamp.Set(percentSpeed);
#endif
}

void IntakeClamp::Stop() {
#ifdef ENABLE_CLAMP_MOTORS
  m_intakeClamp.StopMotor();
#endif
}

void IntakeClamp::EnableBraking(bool value) {
#ifdef ENABLE_CLAMP_MOTORS
  rev::CANSparkMax::IdleMode mode;
  if (value) {
    mode = rev::CANSparkMax::IdleMode::kBrake;
  } else {
    mode = rev::CANSparkMax::IdleMode::kCoast;
  }
  m_intakeClamp.SetIdleMode(mode);
#endif
}

bool IntakeClamp::IsIntakeClampDeployed() {
  // possible function that might need to be implemeneted to prevent the intake
  // from overextending
  return false;
}
