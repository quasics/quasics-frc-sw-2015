// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber() {
  SetName("Climber");
}

void Climber::StartExtending() {
  m_climberLeft.Set(MotorSpeeds::EXTENSION_SPEED);
  m_climberRight.Set(MotorSpeeds::EXTENSION_SPEED);
  m_currentStatus = Movement::eUp;
}

void Climber::StartRetracting() {
  m_climberLeft.Set(MotorSpeeds::RETRACTION_SPEED);
  m_climberRight.Set(MotorSpeeds::RETRACTION_SPEED);
  m_currentStatus = Movement::eDown;
}

void Climber::ExtendOneClimber(bool isLeft) {
  if (isLeft == true) {
    m_climberLeft.Set(MotorSpeeds::EXTENSION_SPEED);
    m_currentStatus = Movement::eUp;
  } else {
    m_climberRight.Set(MotorSpeeds::EXTENSION_SPEED);
    m_currentStatus = Movement::eUp;
  }
}

void Climber::RetractOneClimber(bool isLeft) {
  if (isLeft == true) {
    m_climberLeft.Set(MotorSpeeds::RETRACTION_SPEED);
    m_currentStatus = Movement::eDown;
  } else {
    m_climberRight.Set(MotorSpeeds::RETRACTION_SPEED);
    m_currentStatus = Movement::eDown;
  }
}

void Climber::Stop() {
  m_climberLeft.StopMotor();
  m_climberRight.StopMotor();
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
  /* maybe use later?
frc::SmartDashboard::PutString(
    "Left Climber Limit Switch",
    bottomLimitSwitchLeftClimber.Get() ? "open" : "closed");
frc::SmartDashboard::PutString(
    "Right Climber Limit Switch",
    bottomLimitSwitchRightClimber.Get() ? "open" : "closed");
    */

  frc::SmartDashboard::PutNumber("Left Revolutions", getLeftRevolutions());
  frc::SmartDashboard::PutNumber("Right Revolutions", getRightRevolutions());
}

bool Climber::IsFullyExtended() {
  if (getLeftRevolutions() > 6 && getRightRevolutions() > 6) {
    return true;
  }
  return false;
}

bool Climber::IsFullyRetracted() {
  if (getLeftRevolutions() <= 0 && getRightRevolutions() <= 0) {
    return true;
  }
  return false;
}

double Climber::getLeftRevolutions() {
  // counts per rev is 42 therefore I divide by 42
  return m_leftEncoder.GetPosition() / 42;
}
double Climber::getRightRevolutions() {
  // counts per rev is 42 therefore I divide by 42
  return m_rightEncoder.GetPosition() / 42;
}

void Climber::resetRevolutions() {
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
}
