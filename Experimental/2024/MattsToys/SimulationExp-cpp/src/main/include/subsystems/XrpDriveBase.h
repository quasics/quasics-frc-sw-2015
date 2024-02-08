// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "PreprocessorConfig.h"

#ifdef ENABLE_XRP
#include <frc/xrp/XRPGyro.h>
#include <frc/xrp/XRPMotor.h>

#include "sensors/IGyro.h"
#include "sensors/TrivialEncoder.h"
#include "subsystems/IDrivebase.h"

/**
 * Implements the generic IDrivebase interface in a way that allows for control
 * of an XRP "mini-bot".
 */
class XrpDriveBase : public IDrivebase {
 public:
  XrpDriveBase();

  // Hardware abstraction layer (HAL) functions required for the IDrivebase
  // interface.  (All should be documented in the base class.)
 protected:
  void setMotorVoltages_HAL(units::volt_t leftPower,
                            units::volt_t rightPower) override;

  frc::DifferentialDriveOdometry& getOdometry_HAL() override {
    return m_odometry;
  }

  TrivialEncoder& getLeftEncoder_HAL() override {
    return *m_leftTrivialEncoder;
  }

  TrivialEncoder& getRightEncoder_HAL() override {
    return *m_rightTrivialEncoder;
  }

  IGyro& getGyro_HAL() override {
    return *m_iGyro;
  }

  double getLeftSpeedPercentage_HAL() override {
    return m_leftXrpMotor.Get();
  }
  double getRightSpeedPercentage_HAL() override {
    return m_rightXrpMotor.Get();
  }

  // Data members.  (Mostly, the motors and sensors on the robot, and the
  // "generifying" wrappers for them that are needed by IDrivebase.
 private:
  frc::XRPMotor m_leftXrpMotor{0};
  frc::XRPMotor m_rightXrpMotor{1};
  frc::Encoder m_leftXrpEncoder{4, 5};
  frc::Encoder m_rightXrpEncoder{6, 7};
  frc::XRPGyro m_xrpGyro;

  // Odometry information for the robot.
  frc::DifferentialDriveOdometry m_odometry;

  // "Standardized" wrappers around underlying gyros/encoders.  (Needed by
  // IDrivebase class.)
 private:
  std::unique_ptr<TrivialEncoder> m_leftTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_leftXrpEncoder)};
  std::unique_ptr<TrivialEncoder> m_rightTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_rightXrpEncoder)};
  std::unique_ptr<IGyro> m_iGyro{IGyro::wrapYawGyro(m_xrpGyro)};
};
#endif  // ENABLE_XRP
