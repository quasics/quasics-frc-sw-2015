// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/RobotController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "Constants.h"
#include "sensors/IGyro.h"
#include "sensors/TrivialEncoder.h"
#include "utils/DeadBandEnforcer.h"

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

  frc2::CommandPtr sysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr sysIdDynamic(frc2::sysid::Direction direction);

  void stop() {
    tankDrive(0, 0);
  }

  frc::Pose2d getPose() {
    return getOdometry().GetPose();
  }

  void resetOdometry(frc::Pose2d pose) {
    getOdometry().ResetPosition(getGyro().getRotation2d(),
                                getLeftEncoder().getPosition(),
                                getRightEncoder().getPosition(), pose);
    // ResetEncoders();
  }

  void resetOdometry(frc::Rotation2d& gyroAngle, units::meter_t leftDistance,
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

  frc2::sysid::SysIdRoutine m_sysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                          std::nullopt},
      frc2::sysid::Mechanism{
          [this](units::volt_t volts) {
            // set voltage speed here to left and right
            // motors
            this->setMotorVoltages(volts, volts);
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("drive-left")
                .voltage(getLeftSpeedPercentage() *
                         frc::RobotController::GetBatteryVoltage())
                .position(getLeftEncoder().getPosition())
                .velocity(getLeftEncoder().getVelocity());
            log->Motor("drive-right")
                .voltage(getRightSpeedPercentage() *
                         frc::RobotController::GetBatteryVoltage())
                .position(getRightEncoder().getPosition())
                .velocity(getRightEncoder().getVelocity());
          },
          this}};

 protected:
  static double convertVoltageToPercentSpeed(units::volt_t volts) {
    const double voltageInput = frc::RobotController::GetInputVoltage();
    const double metersPerSec = (volts.value() / voltageInput);
    const double speedPercentage = m_voltageDeadbandEnforcer(
        metersPerSec / RobotConstants::MAX_SPEED.value());
    return speedPercentage;
  }
  virtual void setMotorVoltages(units::volt_t leftPower,
                                units::volt_t rightPower) = 0;

  // Hardware abstraction layer
 protected:
  virtual IGyro& getGyro() = 0;

  virtual void setMotorSpeeds(double leftPercent, double rightPercent) = 0;

  virtual double getLeftSpeedPercentage() = 0;
  virtual double getRightSpeedPercentage() = 0;

  virtual TrivialEncoder& getLeftEncoder() = 0;
  virtual TrivialEncoder& getRightEncoder() = 0;

 private:
  static DeadBandEnforcer m_voltageDeadbandEnforcer;
};