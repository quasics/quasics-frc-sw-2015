// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeRoller.h"

#undef ENABLE_ROLLER_INTAKE

IntakeRoller::IntakeRoller() {}

// This method will be called once per scheduler run
void IntakeRoller::Periodic() {}

void IntakeRoller::SetRollerSpeed(double percentSpeed) {
#ifdef ENABLE_ROLLER_INTAKE
  m_floorRollerPickupMotor.Set(-percentSpeed);
#endif
}

void IntakeRoller::Stop() {
#ifdef ENABLE_ROLLER_INTAKE
  m_floorRollerPickupMotor.Set(0);
#endif
}