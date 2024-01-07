// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#ifdef ENABLE_CTRE
#include <ctre/phoenix6/Pigeon2.hpp>
#endif

#include "Constants.h"
#include "PreprocessorConfig.h"
#include "sensors/OffsetGyro.h"
#include "subsystems/IDrivebase.h"

/**
 * Drive base subsystem for actual FRC hardware, using the motor configuration
 * that Quasics has employed for the last few years (Spark MAX motors at known
 * CAN addresses).
 */
class RealDriveBase : public IDrivebase {
#ifdef ENABLE_CTRE
  using Pigeon2 = ctre::phoenix6::hardware::Pigeon2;
#endif

 public:
  RealDriveBase();

  /**
   * Tell the motors to coast (or brake) if they're not being told how fast to
   * go (e.g., when the robot is disabled, or not being driven in auto mode).
   *
   * @param tf iff true, configure for coast mode; otherwise, for braking
   */
  void enableCoastingMode(bool tf);

  // Methods required by the IDrivebase class.
 protected:
  void setMotorVoltagesImpl(units::volt_t leftPower,
                            units::volt_t rightPower) override;

  frc::DifferentialDriveOdometry& getOdometry() override {
    return m_odometry;
  }

  TrivialEncoder& getLeftEncoder() override {
    return *m_leftTrivialEncoder;
  }

  TrivialEncoder& getRightEncoder() override {
    return *m_rightTrivialEncoder;
  }

  IGyro& getGyro() override {
    return m_offsetGyro;
  }

  // Internal helper functions.
 private:
  void configureMotors();
  void configureEncoders();

  void resetEncoders() {
    m_leftFrontEncoder.SetPosition(0);
    m_leftBackEncoder.SetPosition(0);
    m_rightFrontEncoder.SetPosition(0);
    m_rightBackEncoder.SetPosition(0);
  }

  // Data members.
 private:
  // Drive base motors.
  rev::CANSparkMax m_leftFront{MotorIds::SparkMax::LEFT_FRONT_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFront{MotorIds::SparkMax::RIGHT_FRONT_DRIVE_MOTOR_ID,
                                rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftBack{MotorIds::SparkMax::LEFT_BACK_DRIVE_MOTOR_ID,
                              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightBack{MotorIds::SparkMax::RIGHT_BACK_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};

  // Encoders for each of the motors.
  rev::SparkRelativeEncoder m_leftFrontEncoder =
      m_leftFront.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_rightFrontEncoder =
      m_rightFront.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_leftBackEncoder =
      m_leftBack.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_rightBackEncoder =
      m_rightBack.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  // Motor controller groups, pairing sets on left/right.
  frc::MotorControllerGroup m_leftSide{m_leftFront, m_leftBack};
  frc::MotorControllerGroup m_rightSide{m_rightFront, m_rightBack};

  // Gyro.
  //
  // TODO: Enable using a Pigeon2, once the updated CTRE libraries are
  // available.
#ifdef ENABLE_CTRE
  Pigeon2 m_realGyro;
#else
  frc::ADXRS450_Gyro m_realGyro{frc::SPI::Port::kOnboardCS0};
#endif

  // Odometry information for the robot.
  frc::DifferentialDriveOdometry m_odometry;

  // "Standardized" wrappers around underlying gyros/encoders.  (Needed by
  // IDrivebase class.)
 private:
  /** IGyro wrapper around the real gyro. */
  std::unique_ptr<IGyro> m_trivialGyro{IGyro::wrapGyro(m_realGyro)};

  /**
   * Additional OffsetGyro wrapper around the real gyro, preventing "reset" from
   * affecting all axes.
   */
  OffsetGyro m_offsetGyro{*m_trivialGyro};

  /** Wraps a TrivialEncoder interface around the left encoder. */
  std::unique_ptr<TrivialEncoder> m_leftTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_leftFrontEncoder)};

  /** Wraps a TrivialEncoder interface around the right encoder. */
  std::unique_ptr<TrivialEncoder> m_rightTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_rightFrontEncoder)};
};
