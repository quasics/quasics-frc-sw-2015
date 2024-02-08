// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>

#include "subsystems/IDrivebase.h"

class SimulatedDriveBase : public IDrivebase {
 public:
  SimulatedDriveBase();

  virtual void resetOdometry(frc::Pose2d pose) override {
    IDrivebase::resetOdometry(pose);
    m_drivetrainSimulator.SetPose(pose);
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override {
    IDrivebase::Periodic();
    m_fieldSim.SetRobotPose(m_odometry.GetPose());
  }

  void SimulationPeriodic() override;

  // Hardware abstraction layer (HAL) functions required for the IDrivebase
  // interface.  (All should be documented in the base class.)
 protected:
  void setMotorVoltages_HAL(units::volt_t leftPower,
                            units::volt_t rightPower) override {
    m_leftMotor.SetVoltage(leftPower);
    m_rightMotor.SetVoltage(rightPower);
  }

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
    return m_leftMotor.Get();
  }
  double getRightSpeedPercentage_HAL() override {
    return m_rightMotor.Get();
  }

 private:
  static constexpr units::meter_t kTrackWidth = 0.381_m * 2;
  static constexpr double kWheelRadius = 0.0508;  // meters
  static constexpr int kEncoderResolution = 4096;

  // Note that we'll simply simulate 1 motor on each side.
  frc::PWMSparkMax m_leftMotor{1};
  frc::PWMSparkMax m_rightMotor{3};

  frc::Encoder m_leftEncoder{0, 1};
  frc::Encoder m_rightEncoder{2, 3};

  frc::AnalogGyro m_gyro{0};

  frc::DifferentialDriveOdometry m_odometry;

  // "Standardized" wrappers around underlying gyros/encoders.  (Needed by
  // IDrivebase class.)
 private:
  std::unique_ptr<IGyro> m_iGyro{IGyro::wrapGyro(m_gyro)};
  std::unique_ptr<TrivialEncoder> m_leftTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_leftEncoder)};
  std::unique_ptr<TrivialEncoder> m_rightTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_rightEncoder)};

  // WPILib stuff to help us simulate a drive base on a normal desktop/laptop.
 private:
  frc::sim::AnalogGyroSim m_gyroSim{m_gyro};
  frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
  frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
  frc::Field2d m_fieldSim;
  frc::LinearSystem<2, 2, 2> m_drivetrainSystem =
      frc::LinearSystemId::IdentifyDrivetrainSystem(
          1.98_V / 1_mps, 0.2_V / 1_mps_sq, 1.5_V / 1_mps, 0.3_V / 1_mps_sq);
  frc::sim::DifferentialDrivetrainSim m_drivetrainSimulator{
      m_drivetrainSystem, kTrackWidth, frc::DCMotor::CIM(2), 8, 2_in};
};
