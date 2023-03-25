// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeRoller.h"

#include <iostream>

IntakeRoller::IntakeRoller() {
  SetName("IntakeRoller");

#ifdef ENABLE_ROLLER_INTAKE_MOTORS
  std::cerr << "Intake roller is enabled\n";
#else
  std::cerr << "Intake roller is NOT enabled\n";
#endif
}

// This method will be called once per scheduler run
void IntakeRoller::Periodic() {
}

void IntakeRoller::SetRollerSpeed(double percentSpeed) {
#ifdef ENABLE_ROLLER_INTAKE_MOTORS
  m_floorRollerPickupMotor.Set(-percentSpeed);
#endif
}

void IntakeRoller::Stop() {
#ifdef ENABLE_ROLLER_INTAKE_MOTORS
  m_floorRollerPickupMotor.Set(0);
#endif
}