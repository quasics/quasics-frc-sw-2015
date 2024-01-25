// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "Constants.h"
#include "sensors/IGyro.h"
#include "sensors/TrivialEncoder.h"

class IDrivebase : public frc2::SubsystemBase {
 public:
  IDrivebase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void tankDrive(double leftInputPercent, double rightInputPercent);

 public:
  void arcadeDrive(units::meters_per_second_t fSpeed,
                   units::radians_per_second_t rSpeed) {
    frc::ChassisSpeeds speeds;
    speeds.vx = fSpeed;
    speeds.omega = rSpeed;
    const auto wheelSpeeds = m_kinematics.ToWheelSpeeds(speeds);

    setMotorSpeeds(wheelSpeeds.left / RobotConstants::MAX_SPEED,
                   wheelSpeeds.right / RobotConstants::MAX_SPEED);
  }

  void stop() {
    tankDrive(0, 0);
  }

  frc::Pose2d getPose() {
    return getOdometry().GetPose();
  }

  void ResetOdometry(frc::Pose2d pose) {
    getOdometry().ResetPosition(getGyro().getRotation2d(), 0_m, 0_m, pose);
    ResetEncoders();
  }

  void ResetOdometry(frc::Rotation2d& gyroAngle, units::meter_t leftDistance,
                     units::meter_t rightDistance, frc::Pose2d& pose) {
    getOdometry().ResetPosition(gyroAngle, leftDistance, rightDistance, pose);
  }

  void ResetEncoders() {
    getLeftEncoder().reset();
    getRightEncoder().reset();
  }

  frc::DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return {getLeftEncoder().getVelocity(), getRightEncoder().getVelocity()};
  }

  void updateOdometry() {
    getOdometry().Update(getGyro().getRotation2d().Degrees(),
                         getLeftEncoder().getPosition(),
                         getRightEncoder().getPosition());
  }

  virtual void tankDriveVolts(units::volt_t left, units::volt_t right) = 0;

  virtual frc::DifferentialDriveOdometry& getOdometry() = 0;

 private:
  const frc::DifferentialDriveKinematics m_kinematics{0.558_m};

  // Hardware abstraction layer
 protected:
  virtual IGyro& getGyro() = 0;

  virtual void setMotorSpeeds(double leftPercent, double rightPercent) = 0;

  virtual TrivialEncoder& getLeftEncoder() = 0;
  virtual TrivialEncoder& getRightEncoder() = 0;
};