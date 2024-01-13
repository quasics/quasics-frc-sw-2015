// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

#include <iostream>

class Drivebase : public frc2::SubsystemBase {
 public:
  Drivebase();
  ~Drivebase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Stop();
  void SetMotorSpeed(double leftSpeed, double rightSpeed);
  void ArcadeDrive(double forwardSpeed, double turnSpeed);
  void ResetEncoders();
  double GetLeftEncoderCount();
  double GetRightEncoderCount();
  units::meter_t GetRightEncoderDistance();
  units::meter_t GetLeftEncoderDistance();

 public:
  frc::Pose2d GetPose();
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();
  void ResetOdometry(frc::Pose2d pose);
  void TankDriveVolts(units::volt_t left, units::volt_t right);

 private:
  class RevRoboticsStuff;
  RevRoboticsStuff* const m_revStuff;


  std::unique_ptr<frc::MotorControllerGroup> m_leftMotors;
  std::unique_ptr<frc::MotorControllerGroup> m_rightMotors;

  std::unique_ptr<frc::DifferentialDrive> m_drive;
  };
