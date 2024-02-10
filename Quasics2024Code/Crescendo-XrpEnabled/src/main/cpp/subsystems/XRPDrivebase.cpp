// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/XRPDrivebase.h"

#include <units/length.h>

#ifdef ENABLE_XRP

XRPDrivebase::XRPDrivebase() : m_odometry{frc::Rotation2d(), 0_m, 0_m} {
}

void XRPDrivebase::setMotorVoltages_HAL(units::volt_t leftPower,
                                        units::volt_t rightPower) {
  m_leftXrpMotor.Set(convertVoltageToPercentSpeed(leftPower));
  m_rightXrpMotor.Set(convertVoltageToPercentSpeed(rightPower));
  m_leftXrpMotor.SetVoltage(leftPower);
  m_rightXrpMotor.SetVoltage(rightPower);
}

void XRPDrivebase::setMotorSpeeds_HAL(double leftPercent, double rightPercent) {
  m_leftXrpMotor.SetVoltage(convertPercentSpeedToVoltage(leftPercent));
  m_rightXrpMotor.SetVoltage(convertPercentSpeedToVoltage(rightPercent));
  m_leftXrpMotor.Set(leftPercent);
  m_rightXrpMotor.Set(rightPercent);
}

#endif  // ENABLE_XRP
