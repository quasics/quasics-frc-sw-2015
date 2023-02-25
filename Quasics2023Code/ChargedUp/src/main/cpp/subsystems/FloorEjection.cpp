// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FloorEjection.h"

FloorEjection::FloorEjection() = default;

void FloorEjection::SetFloorEjectionPower(double power) {
  m_floorEjectionMotor.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, power);
}

void FloorEjection::Stop() {
  m_floorEjectionMotor.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, 0);
}

// This method will be called once per scheduler run
void FloorEjection::Periodic() {
}
