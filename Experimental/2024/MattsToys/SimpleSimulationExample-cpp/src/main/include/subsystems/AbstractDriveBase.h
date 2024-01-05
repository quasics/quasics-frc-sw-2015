// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

class AbstractDriveBase : public frc2::SubsystemBase {
 public:
  AbstractDriveBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void tankDrive(double leftInputPercent, double rightInputPercent);

  void stop() {
    tankDrive(0, 0);
  }

  // Hardware abstraction layer
 protected:
  virtual void setMotorSpeeds(double leftPercent, double rightPercent) = 0;
};
