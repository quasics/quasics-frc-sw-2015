// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FloorEjection.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

FloorEjection::FloorEjection() {
  SetName("FloorEjection");
}

void FloorEjection::SetFloorEjectionPower(double power) {
#ifdef ENABLE_FLOOR_EJECTION_MOTOR
  m_floorEjectionMotor.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, power);
#endif
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
            << m_floorEjectionEncoder.GetPositionOffset() << std::endl;
  std::cout << "Am I connected: " << m_floorEjectionEncoder.IsConnected()
            << std::endl;*/
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
#ifdef ENABLE_FLOOR_EJECTION_MOTOR
  m_floorEjectionMotor.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput, 0);
#endif
}

bool FloorEjection::FloorRetracted() {
  // we are assuming that the switch is wired to report true when openend
  return !m_floorRetractionLimitSwitch.Get();
}

void FloorEjection::SetBrakingMode(bool brake) {
#ifdef ENABLE_FLOOR_EJECTION_MOTOR
  if (brake) {
    m_floorEjectionMotor.SetNeutralMode(
        ctre::phoenix::motorcontrol::NeutralMode::Brake);
  } else {
    m_floorEjectionMotor.SetNeutralMode(
        ctre::phoenix::motorcontrol::NeutralMode::Coast);
  }
#endif
}

// This method will be called once per scheduler run
void FloorEjection::Periodic() {
#ifdef ENABLE_FLOOR_EJECTION_ENCODER
  frc::SmartDashboard::PutNumber("Floor ejection position",
                                 m_floorEjectionEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Floor ejection velocity",
                                 m_floorEjectionEncoder.GetRate());

#endif
  frc::SmartDashboard::PutString("Floor Retraction Limit Switch",
                                 FloorRetracted() ? "true" : "false");
  // frc::SmartDashboard::PutBoolean("Floor Retracted", FloorRetracted());
  /*
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
                                   m_floorEjectionEncoder.GetPositionOffset());*/
}
