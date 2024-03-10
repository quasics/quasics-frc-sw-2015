// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeRoller.h"

#include <iostream>

#include "Constants.h"

IntakeRoller::IntakeRoller()
    : m_intake{MotorIds::SparkMax::INTAKE_MOTOR,
               rev::CANSparkMax::MotorType::kBrushless} {
  SetName("IntakeRoller");
}

void IntakeRoller::SetRollerSpeed(double percentSpeed) {
  m_intake.Set(-percentSpeed);
}

void IntakeRoller::Stop() {
  m_intake.Set(0);
}