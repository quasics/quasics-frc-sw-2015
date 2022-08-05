// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber() {
  SetName("Climber");
}

void Climber::StartExtending() {
  // (Matthew): Switch to a named constant for the extension speed. (done)
  //
  // (Matthew): Consider how you should handle things if the arms are
  // already fully extended.(done)

  m_climbers.Set(EXTENSION_SPEED);
  m_currentStatus = Movement::eUp;
}

void Climber::StartRetracting() {
  // (Matthew): Switch to a named constant for the retraction speed. (done)
  //
  // (Matthew): Consider how you should handle things if the arms are
  // already fully retracted.(done)

  m_climbers.Set(RETRACTION_SPEED);
  m_currentStatus = Movement::eDown;
}

void Climber::ExtendOneClimber(bool isLeft) {
  if (isLeft == true) {
    m_climberLeft.Set(EXTENSION_SPEED);
    m_currentStatus = Movement::eUp;
  } else {
    m_climberRight.Set(EXTENSION_SPEED);
    m_currentStatus = Movement::eUp;
  }
}

void Climber::RetractOneClimber(bool isLeft) {
  if (isLeft == true) {
    m_climberLeft.Set(RETRACTION_SPEED);
    m_currentStatus = Movement::eDown;
  } else {
    m_climberRight.Set(RETRACTION_SPEED);
    m_currentStatus = Movement::eDown;
  }
}

void Climber::Stop() {
  m_climbers.StopMotor();
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
  frc::SmartDashboard::PutString(
      "Left Climber Limit Switch",
      bottomLimitSwitchLeftClimber.Get() ? "open" : "closed");
  frc::SmartDashboard::PutString(
      "Right Climber Limit Switch",
      bottomLimitSwitchRightClimber.Get() ? "open" : "closed");
  // if (GetCurrentStatus() == Movement::eUp) {
  //   if (IsFullyExtended()) {
  //     Stop();
  //   }
  // } else {
  //   if (IsFullyRetracted()) {
  //     Stop();
  //   }
  // }
}

bool Climber::IsFullyExtended() {
  // (Matthew): Implement this method. (done)
  return topLimitSwitch.Get();
}

bool Climber::IsFullyRetracted() {
  // (Matthew): Implement this method. (done)
  return !bottomLimitSwitchLeftClimber.Get() ||
         !bottomLimitSwitchRightClimber.Get();
}
