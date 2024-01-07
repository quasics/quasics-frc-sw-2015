// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/xrp/XRPMotor.h>

#include "subsystems/AbstractDriveBase.h"

class XrpDriveBase : public AbstractDriveBase {
 public:
  XrpDriveBase();

  // Hardware abstraction layer
 protected:
  void setMotorSpeeds(double leftPercent, double rightPercent) override;

 private:
  frc::XRPMotor m_leftXrpMotor{0};
  frc::XRPMotor m_rightXrpMotor{1};
};
