// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include <iostream>

IntakeRoller::IntakeRoller() {
  SetName("IntakeRoller");
}

// This method will be called once per scheduler run
void IntakeRoller::Periodic() {
}

void IntakeRoller::SetRollerSpeed(double percentSpeed) {
  m_floorRollerPickupMotor.Set(-percentSpeed);
}

void IntakeRoller::Stop() {
  m_floorRollerPickupMotor.Set(0);
}