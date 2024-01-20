// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RealDrivebase.h"

RealDrivebase::RealDrivebase() {
  SetName("RealDrivebase");
  // This is where we'd do any necessary motor configuration (e.g., setting some
  // as "inverted", etc.).
}

void RealDrivebase::setMotorSpeeds(double leftPercent, double rightPercent) {
  m_leftBack.Set(leftPercent);
  m_rightBack.Set(rightPercent);
}

void RealDrivebase::tankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftBack.SetVoltage(left);
  m_rightBack.SetVoltage(right);
}