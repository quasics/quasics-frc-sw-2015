// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() {
  SetName("Shooter");
}

void Shooter::SetFlywheelSpeed(double percentSpeed) {
  m_flyWheel.Set(percentSpeed);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
}

void Shooter::ExtendLinearActuators() {
  m_leftPositionServo.Set(1.00);
  m_rightPositionServo.Set(1.00);
}

void Shooter::RetractLinearActuators() {
  m_leftPositionServo.Set(0.00);
  m_rightPositionServo.Set(0.00);
}
