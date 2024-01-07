// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/XrpDriveBase.h"

XrpDriveBase::XrpDriveBase() {
  SetName("XrpDriveBase");

  // Per docs: "The right motor will spin in a backward direction when positive
  // output is applied. Thus the corresponding motor controller needs to be
  // inverted in robot code."
  m_rightXrpMotor.SetInverted(true);
  m_leftXrpMotor.SetInverted(false);
}

void XrpDriveBase::setMotorSpeeds(double leftPercent, double rightPercent) {
  m_leftXrpMotor.Set(leftPercent);
  m_rightXrpMotor.Set(rightPercent);
}
