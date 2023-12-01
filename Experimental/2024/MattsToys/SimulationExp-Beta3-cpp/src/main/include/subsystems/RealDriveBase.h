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
#include "subsystems/IDrivebase.h"

class RealDriveBase : public frc2::SubsystemBase, public IDrivebase {
 public:
  RealDriveBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override {
    IDrivebase::Periodic();
  }

 protected:
  void setMotorVoltages(units::volt_t leftPower,
                        units::volt_t rightPower) override {
    // TODO: implement this
  }

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
    return *m_trivialGyro;
  }

 private:
  rev::CANSparkMax m_leftFront{MotorIds::SparkMax::LEFT_FRONT_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFront{MotorIds::SparkMax::RIGHT_FRONT_DRIVE_MOTOR_ID,
                                rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftBack{MotorIds::SparkMax::LEFT_BACK_DRIVE_MOTOR_ID,
                              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightBack{MotorIds::SparkMax::RIGHT_BACK_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};

  // encoders for each of the motors.
  rev::SparkMaxRelativeEncoder m_leftFrontEncoder = m_leftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightFrontEncoder = m_rightFront.GetEncoder();
  rev::SparkMaxRelativeEncoder m_leftBackEncoder = m_leftBack.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightBackEncoder = m_rightBack.GetEncoder();

  std::unique_ptr<frc::MotorControllerGroup> m_leftSide;
  std::unique_ptr<frc::MotorControllerGroup> m_rightSide;

  std::unique_ptr<TrivialEncoder> m_leftTrivialEncoder{
      FunctionalTrivialEncoder::forRevEncoder(m_leftFrontEncoder)};
  std::unique_ptr<TrivialEncoder> m_rightTrivialEncoder{
      FunctionalTrivialEncoder::forRevEncoder(m_rightFrontEncoder)};

  std::unique_ptr<frc::DifferentialDrive> m_drive;

  frc::ADXRS450_Gyro m_realGyro{frc::SPI::Port::kOnboardCS0};
  std::unique_ptr<IGyro> m_trivialGyro{IGyro::wrapGyro(m_realGyro)};

  frc::DifferentialDriveOdometry m_odometry;
};
