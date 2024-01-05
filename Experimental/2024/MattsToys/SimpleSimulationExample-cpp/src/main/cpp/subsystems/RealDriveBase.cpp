// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RealDriveBase.h"

RealDriveBase::RealDriveBase() {
  SetName("RealDriveBase");
  // This is where we'd do any necessary motor configuration (e.g., setting some
  // as "inverted", etc.).
}

void RealDriveBase::setMotorSpeeds(double leftPercent, double rightPercent) {
  m_leftSide.Set(leftPercent);
  m_rightSide.Set(rightPercent);
}