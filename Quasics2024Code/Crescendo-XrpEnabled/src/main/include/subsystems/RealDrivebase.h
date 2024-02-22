// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include "sensors/OffsetGyro.h"
#include "subsystems/IDrivebase.h"

class RealDrivebase : public IDrivebase {
 public:
  RealDrivebase();

  // Hardware abstraction layer
 protected:
  void setMotorSpeeds_HAL(double leftPercent, double rightPercent) override;

  void setMotorVoltages_HAL(units::volt_t leftPower,
                            units::volt_t rightPower) override;

  IGyro& getGyro_HAL() override {
    return m_offsetGyro;
  }

  TrivialEncoder& getLeftEncoder_HAL() override {
    return *m_leftTrivialEncoder;
  }

  TrivialEncoder& getRightEncoder_HAL() override {
    return *m_rightTrivialEncoder;
  }

  double getLeftSpeedPercentage_HAL() override {
    return m_leftBack.Get();
  }
  double getRightSpeedPercentage_HAL() override {
    return m_rightBack.Get();
  }

  frc::DifferentialDriveOdometry& getOdometry_HAL() override {
    return m_odometry;
  }

 private:
  void configureEncoders();

  void resetEncoders() {
    m_leftBackEncoder.SetPosition(0);
    m_rightBackEncoder.SetPosition(0);
  }

  rev::CANSparkMax m_leftBack;
  rev::CANSparkMax m_rightBack;
  rev::CANSparkMax m_leftBackFollower;
  rev::CANSparkMax m_rightBackFollower;

  rev::SparkRelativeEncoder m_leftBackEncoder =
      m_leftBack.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_rightBackEncoder =
      m_rightBack.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  /** Wraps a TrivialEncoder interface around the left encoder. */
  std::unique_ptr<TrivialEncoder> m_leftTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_leftBackEncoder)};

  /** Wraps a TrivialEncoder interface around the right encoder. */
  std::unique_ptr<TrivialEncoder> m_rightTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_rightBackEncoder)};

  ctre::phoenix6::hardware::Pigeon2 m_realGyro;

  std::unique_ptr<IGyro> m_trivialGyro{IGyro::wrapGyro(m_realGyro)};

  OffsetGyro m_offsetGyro{*m_trivialGyro};

  frc::DifferentialDriveOdometry m_odometry{0_rad, 0_m, 0_m};
};
