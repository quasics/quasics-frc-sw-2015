// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "subsystems/IDrivebase.h"

class SimulatedDriveBase : public frc2::SubsystemBase, public IDrivebase {
 public:
  SimulatedDriveBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override {
    IDrivebase::Periodic();
  }

 protected:
  void setMotorVoltages(double leftPower, double rightPower) override {
    // TODO: implement this
  }

  frc::DifferentialDriveOdometry& getOdometry() override {
    return m_odometry;
  }

  TrivialEncoder& getLeftEncoder() override {
    // TODO: Implement this.
    return TrivialEncoder::getNullEncoder();
  }

  TrivialEncoder& getRightEncoder() override {
    // TODO: Implement this.
    return TrivialEncoder::getNullEncoder();
  }

  IGyro& getGyro() override {
    // TODO: Implement this
    return IGyro::getNullGyro();
  }

 private:
  frc::DifferentialDriveOdometry m_odometry;
};
