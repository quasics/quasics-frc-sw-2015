// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FloorEjection.h"

#include <frc/smartdashboard/SmartDashboard.h>

FloorEjection::FloorEjection() = default;

void FloorEjection::SetFloorEjectionPower(double power) {
  m_floorEjectionMotor.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, power);
}

double FloorEjection::GetPosition() {
#ifdef ENABLE_FLOOR_EJECTION_ENCODER
  return m_floorEjectionEncoder.GetDistance();
#endif
  return m_floorEjectionEncoder.GetAbsolutePosition();
}

void FloorEjection::ResetEncoder() {
  m_floorEjectionEncoder.Reset();
}

double FloorEjection::GetVelocity() {
#ifdef ENABLE_FLOOR_EJECTION_ENCODER
  return m_floorEjectionEncoder.GetRate();
#endif
  return 0;
}

void FloorEjection::Stop() {
  m_floorEjectionMotor.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, 0);
}

// This method will be called once per scheduler run
void FloorEjection::Periodic() {
#ifdef ENABLE_FLOOR_EJECTION_ENCODER
  frc::SmartDashboard::PutNumber("Floor ejection position",
                                 m_floorEjectionEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Floor ejection velocity",
                                 m_floorEjectionEncoder.GetRate());
#endif
  frc::SmartDashboard::PutNumber("Encoder Position", GetPosition());
}
