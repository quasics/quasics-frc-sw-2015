// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>

#include "subsystems/IDrivebase.h"

class SimulatedDrivebase : public IDrivebase {
 public:
  SimulatedDrivebase();

  // Hardware abstraction layer
 protected:
  void setMotorSpeeds(double leftPercent, double rightPercent) override;

  void tankDriveVolts(units::volt_t left, units::volt_t right) override;
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  TrivialEncoder& getLeftEncoder() override {
    return *m_leftTrivialEncoder;
  }

  TrivialEncoder& getRightEncoder() override {
    return *m_rightTrivialEncoder;
  }

  frc::DifferentialDriveOdometry& getOdometry() override {
    return m_odometry;
  }

  IGyro& getGyro() override {
    return *m_trivialGyro;
  }

 public:
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  static constexpr units::meter_t kTrackWidth = 0.381_m * 2;
  static constexpr units::meter_t kWheelRadius = 0.0508_m;
  static constexpr int kEncoderResolution = 4096;

  // Note that we'll simply simulate 1 motor on each side.
  frc::PWMSparkMax m_leftMotor{1};
  frc::PWMSparkMax m_rightMotor{3};

  frc::Encoder m_leftEncoder{0, 1};
  frc::Encoder m_rightEncoder{2, 3};

  /** Wraps a TrivialEncoder interface around the left encoder. */
  std::unique_ptr<TrivialEncoder> m_leftTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_leftEncoder)};

  std::unique_ptr<TrivialEncoder> m_rightTrivialEncoder{
      TrivialEncoder::wrapEncoder(m_rightEncoder)};

  frc::AnalogGyro m_gyro{0};

  std::unique_ptr<IGyro> m_trivialGyro{IGyro::wrapGyro(m_gyro)};

  frc::DifferentialDriveOdometry m_odometry{0_deg, 0_m, 0_m};

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
