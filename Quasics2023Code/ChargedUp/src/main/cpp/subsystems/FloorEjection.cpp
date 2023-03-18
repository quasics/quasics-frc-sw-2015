// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FloorEjection.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

FloorEjection::FloorEjection() = default;

void FloorEjection::SetFloorEjectionPower(double power) {
  m_floorEjectionMotor.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, power);
}

double FloorEjection::GetPosition() {
#ifdef ENABLE_FLOOR_EJECTION_ENCODER
  return m_floorEjectionEncoder.GetDistance();
#endif
  /*std::cout << "Calling the Function" << std::endl;
  std::cout << "Absolute Position: "
            << m_floorEjectionEncoder.GetAbsolutePosition() << std::endl;
  std::cout << "Get Function: " << m_floorEjectionEncoder.Get().value()
            << std::endl;
  std::cout << "GetDistance Function: " << m_floorEjectionEncoder.GetDistance()
            << std::endl;
  std::cout << "GetPositionOffset Function: "
            << m_floorEjectionEncoder.GetPositionOffset() << std::endl;*/
  std::cout << "Am I connected: " << m_floorEjectionEncoder.IsConnected()
            << std::endl;
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
  frc::SmartDashboard::PutNumber("Is it Connected",
                                 m_floorEjectionEncoder.IsConnected());
  frc::SmartDashboard::PutNumber("Encoder Position", GetPosition());
  frc::SmartDashboard::PutNumber("Absolute Position: ",
                                 m_floorEjectionEncoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Get Function: ",
                                 m_floorEjectionEncoder.Get().value());
  frc::SmartDashboard::PutNumber("GetDistance Function: ",
                                 m_floorEjectionEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("GetPositionOffset Function: ",
                                 m_floorEjectionEncoder.GetPositionOffset());
}
