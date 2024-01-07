// Copyright (c) FIRST and other WPILib contributors.
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

#include "subsystems/AbstractDriveBase.h"

class SimulatedDriveBase : public AbstractDriveBase {
 public:
  SimulatedDriveBase();

  // Hardware abstraction layer (from AbstractDriveBase)
 protected:
   void setMotorSpeeds(double leftPercent, double rightPercent) override;

  // Functions defined in the frc::SubsystemBase class
 public:
  void Periodic() override;
  void SimulationPeriodic() override;

  // Data members
 private:
  static constexpr units::meter_t kTrackWidth = 0.381_m * 2;
  static constexpr units::meter_t kWheelRadius = 0.0508_m;
  static constexpr int kEncoderResolution = 4096;

  // Note that we'll simply simulate 1 motor on each side.
  frc::PWMSparkMax m_leftMotor{1};
  frc::PWMSparkMax m_rightMotor{3};

  frc::Encoder m_leftEncoder{0, 1};
  frc::Encoder m_rightEncoder{2, 3};

  frc::AnalogGyro m_gyro{0};

  frc::DifferentialDriveOdometry m_odometry{0_deg, 0_m, 0_m};

  // Simulation objects help us simulate our robot (including some "magic numbers")
 private:
  frc::sim::AnalogGyroSim m_gyroSim{m_gyro};
  frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
  frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};

  // Data for smart dashboard, representing the field and elements on it (including the robot)
  frc::Field2d m_fieldSim;

  // Mathematical description of our drive train.
  frc::LinearSystem<2, 2, 2> m_drivetrainSystem =
      frc::LinearSystemId::IdentifyDrivetrainSystem(
          1.98_V / 1_mps, 0.2_V / 1_mps_sq, 1.5_V / 1_mps, 0.3_V / 1_mps_sq);
  
  // Handles simulation of the drive train (calculating positions/angles, etc.)
  frc::sim::DifferentialDrivetrainSim m_drivetrainSimulator{
      m_drivetrainSystem, kTrackWidth, frc::DCMotor::CIM(2), 8, 2_in};
};
