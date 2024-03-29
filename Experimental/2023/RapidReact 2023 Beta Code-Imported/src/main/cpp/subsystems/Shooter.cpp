// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() {
  SetName("Shooter");

  m_flyWheel.SetInverted(false);
}

void Shooter::SetFlywheelSpeed(double percentSpeed) {
  m_flyWheel.Set(percentSpeed);
}

void Shooter::SetRollerSpeed(double speed) {
  m_rearRoller.Set(speed);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
}
