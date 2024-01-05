// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/AbstractDriveBase.h"

class RealDriveBase : public AbstractDriveBase {
 public:
  RealDriveBase();

  // Hardware abstraction layer
 protected:
  void setMotorSpeeds(double leftPercent, double rightPercent) override;

 private:
  // TODO: Add the real motors (e.g., CANSparkMax, etc.) and "wire them in"
};
