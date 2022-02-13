// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber() {
  m_climbers.reset(
      new frc::MotorControllerGroup(m_climberLeft, m_climberRight));
}

void Climber::StartExtending() {
  // TODO(Matthew): Switch to a named constant for the extension speed.
  m_climbers->Set(0.25);
  m_currentStatus = Movement::eUp;
}

void Climber::StartRetracting() {
  // TODO(Matthew): Switch to a named constant for the retraction speed.
  m_climbers->Set(-0.25);
  m_currentStatus = Movement::eDown;
}

void Climber::Stop() {
  m_climbers->StopMotor();
  m_currentStatus = Movement::eStopped;
}

void Climber::EnableBraking(bool value) {
  rev::CANSparkMax::IdleMode mode;
  if (value) {
    mode = rev::CANSparkMax::IdleMode::kBrake;
  } else {
    mode = rev::CANSparkMax::IdleMode::kCoast;
  }

  // Apply the mode to the climber motors.
  m_climberLeft.SetIdleMode(mode);
  m_climberRight.SetIdleMode(mode);
}

Climber::Movement Climber::GetCurrentStatus() {
  return m_currentStatus;
}
// This method will be called once per scheduler run
void Climber::Periodic() {
}
