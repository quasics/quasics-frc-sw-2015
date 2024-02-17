// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

#include <units/time.h>

Shooter::Shooter() {
  SetName("Shooter");
}

void Shooter::SetFlywheelSpeed(double percentSpeed) {
  m_flyWheel.Set(percentSpeed);
  m_flyWheelTwo.Set(-percentSpeed);
}