// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/BuiltInAccelerometer.h>
#include <frc/Encoder.h>
#include <frc/Spark.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/length.h>

// Defining a common interface for Romi and Mae drive base code.
#include "../../../../Common2021/CommonDriveSubsystem.h"
#include "sensors/RomiGyro.h"

class Drivetrain : public CommonDriveSubsystem {
 public:
  Drivetrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //
  // Methods from CommonDriveSubsystem
 public:
  // Documented in base class.
  void Stop() override {
    m_drive.StopMotor();
  }

  // Documented in base class.
  void ArcadeDrive(double xaxisSpeed, double zaxisRotate,
                   bool squareInputs = true) override;

  // Documented in base class.
  virtual void CurvatureDrive(double xaxisSpeed, double zaxisRotate,
                              bool isQuickTurn) override;

  // Documented in base class.
  void TankDrive(double leftSpeed, double rightSpeed) override;

  // Documented in base class.
  void ResetEncoders() override;

  // Documented in base class.
  double GetLeftEncoderCount() override;

  // Documented in base class.
  double GetRightEncoderCount() override;

  // Documented in base class.
  units::meter_t GetLeftDistance() override;

  // Documented in base class.
  units::meter_t GetRightDistance() override;

  // Documented in base class.
  units::meter_t GetAverageDistance() override;

  // Documented in base class.
  void ResetGyro() override;

  // Documented in base class.
  frc::Gyro& GetZAxisGyro() override {
    return m_gyro.GetGyroZ();
  }

  void AddToShuffleboard(wpi::StringRef label, frc::Sendable* data) override;

  //
  // Additional functions to support trajectory-following.
 public:
  frc::Pose2d GetPose() override;

  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds() override;

  void ResetOdometry(frc::Pose2d pose) override;

  void TankDriveVolts(units::volt_t left, units::volt_t right) override;

  units::meter_t GetTrackWidth() override {
    return kTrackwidthMeters;
  }

  //
  // Additional methods (should not be used in common commands).
 public:
  /**
   * Returns the acceleration along the X-axis, in Gs.
   */
  double GetAccelX();

  /**
   * Returns the acceleration along the Y-axis, in Gs.
   */
  double GetAccelY();

  /**
   * Returns the acceleration along the Z-axis, in Gs.
   */
  double GetAccelZ();

  /**
   * Returns the current angle of the Romi around the X-axis, in degrees.
   */
  double GetGyroAngleX();

  /**
   * Returns the current angle of the Romi around the Y-axis, in degrees.
   */
  double GetGyroAngleY();

  /**
   * Returns the current angle of the Romi around the Z-axis, in degrees.
   */
  double GetGyroAngleZ();

 private:
  /** Sets up reporting of drive base data on the Shuffleboard interface. */
  void ConfigureShuffleboard();

  /** Updates reporting of drive base data on the Shuffleboard interface. */
  void UpdateShuffleboard();

  static constexpr double kCountsPerRevolution = 1440.0;
  static constexpr units::meter_t kWheelDiameter = 70_mm;
  static constexpr units::meter_t kTrackwidthMeters{0.142072613};

 private:
  frc::Spark m_leftMotor{0};
  frc::Spark m_rightMotor{1};

  frc::Encoder m_leftEncoder{4, 5};
  frc::Encoder m_rightEncoder{6, 7};

  frc::DifferentialDrive m_drive{m_leftMotor, m_rightMotor};

  RomiGyro m_gyro;
  frc::BuiltInAccelerometer m_accelerometer;

  frc::DifferentialDriveOdometry m_odometry;

  // Used to display information about the drive base on the shuffleboard.
  nt::NetworkTableEntry leftDistance, rightDistance, leftSpeed, rightSpeed,
      rotation;
};
