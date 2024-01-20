// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

class IDrivebase : public frc2::SubsystemBase {
 public:
  IDrivebase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void tankDrive(double leftInputPercent, double rightInputPercent);

  void arcadeDrive(double forwardInputPercent, double rotationInputPercent);

  void stop() {
    tankDrive(0, 0);
  }

  // Hardware abstraction layer
 protected:
  virtual void setMotorSpeeds(double leftPercent, double rightPercent) = 0;
};