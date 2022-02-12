// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber() {
  m_Climbers.reset(
      new frc::MotorControllerGroup(m_ClimberLeft, m_ClimberRight));
}

void Climber::StartExtracting() {
  m_Climbers->Set(0.25);
  currentStatus = Movement::eUp;
}

void Climber::StartRetracting() {
  m_Climbers->Set(-0.25);
  currentStatus = Movement::eDown;
}

void Climber::Stop() {
  m_Climbers->Set(0);
  currentStatus = Movement::eStopped;
}

void Climber::EnableBrakeing(bool) {
}

Climber::Movement Climber::GetCurrentStatus() {
  return currentStatus;
}
// This method will be called once per scheduler run
void Climber::Periodic() {
}
