// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "subsystems/IDrivebase.h"

class RealDrivebase : public IDrivebase {
 public:
  RealDrivebase();

  // Hardware abstraction layer
 protected:
  void setMotorSpeeds(double leftPercent, double rightPercent) override;

  void tankDriveVolts(units::volt_t left, units::volt_t right) override;

 private:
  // TODO: Add the real motors (e.g., CANSparkMax, etc.) and "wire them in"
  // Drive base motors.
  // rev::CANSparkMax m_leftFront{MotorIds::SparkMax::LEFT_FRONT_DRIVE_MOTOR_ID,
  // rev::CANSparkMax::MotorType::kBrushless
  // };
  // rev::CANSparkMax
  // m_rightFront{MotorIds::SparkMax::RIGHT_FRONT_DRIVE_MOTOR_ID,
  // rev::CANSparkMax::MotorType::kBrushless
  // };
  rev::CANSparkMax m_leftBack{MotorIds::SparkMax::LEFT_BACK_DRIVE_MOTOR_ID,
                              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightBack{MotorIds::SparkMax::RIGHT_BACK_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};

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

  frc::DifferentialDriveOdometry m_odometry{0_rad, 0_m, 0_m};

  TrivialEncoder& getLeftEncoder() override {
    return *m_leftTrivialEncoder;
  }

  TrivialEncoder& getRightEncoder() override {
    return *m_rightTrivialEncoder;
  }

  frc::DifferentialDriveOdometry& getOdometry() override {
    return m_odometry;
  }

  // Motor controller groups, pairing sets on left/right.
  // frc::MotorControllerGroup m_leftSide{m_leftFront, m_leftBack};
  // frc::MotorControllerGroup m_rightSide{m_rightFront, m_rightBack};
};
