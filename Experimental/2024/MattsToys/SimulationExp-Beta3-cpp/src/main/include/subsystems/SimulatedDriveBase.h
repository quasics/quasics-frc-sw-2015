// Copyright (c) FIRST and other WPILib contributors.
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

class SimulatedDriveBase : public frc2::SubsystemBase, public IDrivebase {
 public:
  SimulatedDriveBase();

  virtual void resetOdometry(frc::Pose2d pose) {
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

 protected:
  void setMotorVoltages(units::volt_t leftPower,
                        units::volt_t rightPower) override {
    m_leftGroup.SetVoltage(leftPower);
    m_rightGroup.SetVoltage(rightPower);
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
    return *m_iGyro;
  }

 private:
  static constexpr units::meter_t kTrackWidth = 0.381_m * 2;
  static constexpr double kWheelRadius = 0.0508;  // meters
  static constexpr int kEncoderResolution = 4096;

  frc::PWMSparkMax m_leftLeader{1};
  frc::PWMSparkMax m_leftFollower{2};
  frc::PWMSparkMax m_rightLeader{3};
  frc::PWMSparkMax m_rightFollower{4};

  frc::MotorControllerGroup m_leftGroup{m_leftLeader, m_leftFollower};
  frc::MotorControllerGroup m_rightGroup{m_rightLeader, m_rightFollower};

  frc::Encoder m_leftEncoder{0, 1};
  frc::Encoder m_rightEncoder{2, 3};

  frc::AnalogGyro m_gyro{0};

  frc::DifferentialDriveOdometry m_odometry;

  std::unique_ptr<IGyro> m_iGyro{IGyro::wrapGyro(m_gyro)};
  std::unique_ptr<TrivialEncoder> m_leftTrivialEncoder{
      FunctionalTrivialEncoder::forWpiLibEncoder(m_leftEncoder)};
  std::unique_ptr<TrivialEncoder> m_rightTrivialEncoder{
      FunctionalTrivialEncoder::forWpiLibEncoder(m_rightEncoder)};

  // Simulation classes help us simulate our robot
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
