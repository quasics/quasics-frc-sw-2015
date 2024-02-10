// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ConditionalCompileFlags.h"

#ifdef ENABLE_XRP

#include <frc/xrp/XRPGyro.h>
#include <frc/xrp/XRPMotor.h>

#include "sensors/IGyro.h"
#include "sensors/TrivialEncoder.h"
#include "subsystems/IDrivebase.h"

class XRPDrivebase : public IDrivebase {
 public:
  XRPDrivebase();

 protected:
  virtual void setMotorVoltages_HAL(units::volt_t leftPower,
                                    units::volt_t rightPower) override;
  virtual frc::DifferentialDriveOdometry& getOdometry_HAL() override {
    return m_odometry;
  }
  virtual IGyro& getGyro_HAL() override {
    return *m_iGyro;
  }

  virtual void setMotorSpeeds_HAL(double leftPercent,
                                  double rightPercent) override;

  virtual double getLeftSpeedPercentage_HAL() override {
    return m_leftXrpMotor.Get();
  }
  virtual double getRightSpeedPercentage_HAL() override {
    return m_rightXrpMotor.Get();
  }

  virtual TrivialEncoder& getLeftEncoder_HAL() override {
    return *m_leftTrivialEncoder;
  }
  virtual TrivialEncoder& getRightEncoder_HAL() override {
    return *m_rightTrivialEncoder;
  }

 private:
  frc::XRPMotor m_leftXrpMotor{0};
  frc::XRPMotor m_rightXrpMotor{1};
  frc::Encoder m_leftXrpEncoder{4, 5};
  frc::Encoder m_rightXrpEncoder{6, 7};
  frc::XRPGyro m_xrpGyro;

  // Odometry information for the robot.
  frc::DifferentialDriveOdometry m_odometry;

 private:
  std::unique_ptr<TrivialEncoder> m_leftTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_leftXrpEncoder)};
  std::unique_ptr<TrivialEncoder> m_rightTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_rightXrpEncoder)};
  std::unique_ptr<IGyro> m_iGyro{IGyro::wrapYawGyro(m_xrpGyro)};
};
#endif  // ENABLE_XRP
