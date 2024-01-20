// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>

#include "sensors/TrivialEncoder.h"

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

  frc::Pose2d getPose() {
    return getOdometry().GetPose();
  }

  void ResetOdometry(frc::Pose2d pose) {
    getOdometry().ResetPosition(0_deg, 0_m, 0_m, pose);
  }

  frc::DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return {getLeftEncoder().getVelocity(), getRightEncoder().getVelocity()};
  }

  virtual void tankDriveVolts(units::volt_t left, units::volt_t right) = 0;

  // Hardware abstraction layer
 protected:
  virtual frc::DifferentialDriveOdometry& getOdometry() = 0;

  virtual void setMotorSpeeds(double leftPercent, double rightPercent) = 0;

  virtual TrivialEncoder& getLeftEncoder() = 0;
  virtual TrivialEncoder& getRightEncoder() = 0;
};