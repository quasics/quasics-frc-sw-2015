// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "sensors/OffsetGyro.h"
#include "subsystems/IDrivebase.h"

class RealDriveBase : public IDrivebase {
 public:
  RealDriveBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override {
    IDrivebase::Periodic();
  }

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
  rev::SparkMaxRelativeEncoder m_leftFrontEncoder = m_leftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightFrontEncoder = m_rightFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_leftBackEncoder = m_leftBack.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightBackEncoder = m_rightBack.GetEncoder();

  // Motor controller groups, pairing sets on left/right.
  frc::MotorControllerGroup m_leftSide{m_leftFront, m_leftBack};
  frc::MotorControllerGroup m_rightSide{m_rightFront, m_rightBack};

  // Gyro.
  frc::ADXRS450_Gyro m_realGyro{frc::SPI::Port::kOnboardCS0};

  // Odometry information for the robot.
  frc::DifferentialDriveOdometry m_odometry;

  // Standardized wrappers around underlying gyros/encoders.  (Needed by
  // IDrivebase class.)
  std::unique_ptr<IGyro> m_trivialGyro{IGyro::wrapGyro(m_realGyro)};
  /**
   * OffsetGyro wrapper around the real gyro, preventing "reset" from affecting
   * all axes.
   */
  OffsetGyro m_offsetGyro{*m_trivialGyro};
  std::unique_ptr<TrivialEncoder> m_leftTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_leftFrontEncoder)};
  std::unique_ptr<TrivialEncoder> m_rightTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_rightFrontEncoder)};
};
