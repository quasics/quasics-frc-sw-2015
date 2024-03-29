// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

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
                   units::radians_per_second_t rSpeed);

  frc2::CommandPtr sysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr sysIdDynamic(frc2::sysid::Direction direction);

  void stop() {
    tankDrive(0, 0);
  }

  frc::Pose2d getPose() {
    return getOdometry_HAL().GetPose();
  }

  void resetOdometry(frc::Pose2d pose) {
    // Pass through to the more complicated version.
    resetOdometry(getGyro_HAL().getRotation2d(),
                  getLeftEncoder_HAL().getPosition(),
                  getRightEncoder_HAL().getPosition(), pose);
    // resetEncoders();
  }

  void resetOdometry(frc::Rotation2d gyroAngle, units::meter_t leftDistance,
                     units::meter_t rightDistance, frc::Pose2d& pose) {
    getOdometry_HAL().ResetPosition(gyroAngle, leftDistance, rightDistance,
                                    pose);
  }

  void resetEncoders() {
    getLeftEncoder_HAL().reset();
    getRightEncoder_HAL().reset();
  }

  frc::DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return {getLeftEncoder_HAL().getVelocity(),
            getRightEncoder_HAL().getVelocity()};
  }

  void updateOdometry() {
    getOdometry_HAL().Update(getGyro_HAL().getRotation2d().Degrees(),
                             getLeftEncoder_HAL().getPosition(),
                             getRightEncoder_HAL().getPosition());
  }

  units::degree_t getYaw() {
    // getPose() angle returned is between -180 and 180. This function gets the
    // raw gyro angle without wrapping it aroud those values
    return getGyro_HAL().getRotation2d().Degrees();
  }

  // This is a direct pass-through, because HAL functions should *never* be used
  // directly by external code.
  void setMotorVoltages(units::volt_t leftPower, units::volt_t rightPower) {
    setMotorVoltages_HAL(leftPower, rightPower);
  }

  // This is a direct pass-through, because HAL functions should *never* be used
  // directly by external code, and because we don't want to let external code
  // *modify* the odometry data outside of our control, so we want to ensure
  // that callers only get a *copy* of the odometry data, rather than a
  // reference to our internal state that they can change.
  //
  // @see https://en.wikipedia.org/wiki/Information_hiding
  frc::DifferentialDriveOdometry getOdometry() {
    return getOdometry_HAL();
  }

 private:
  frc2::sysid::SysIdRoutine m_sysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                          std::nullopt},
      frc2::sysid::Mechanism{
          [this](units::volt_t volts) {
            // set voltage speed here to left and right
            // motors
            this->setMotorVoltages_HAL(volts, volts);
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("drive-left")
                .voltage(getLeftVoltage_HAL())
                .position(getLeftEncoder_HAL().getPosition())
                .velocity(getLeftEncoder_HAL().getVelocity());
            log->Motor("drive-right")
                .voltage(getLeftVoltage_HAL())
                .position(getRightEncoder_HAL().getPosition())
                .velocity(getRightEncoder_HAL().getVelocity());
          },
          this}};

 protected:
  static double convertVoltageToPercentSpeed(units::volt_t volts);

  static units::volt_t convertPercentSpeedToVoltage(double percentSpeed);

  // Hardware abstraction layer functions.
  //
  // Note that these should *never* be called directly by code from outside an
  // IDrivebase.  At most, we should provide simple "pass-through" methods that
  // clients can use, and which will call these directly.
  //
  // @see https://en.wikipedia.org/wiki/Hardware_abstraction
  // @see https://stackify.com/oop-concept-abstraction/
  // @see https://stackify.com/oop-concept-for-beginners-what-is-encapsulation/
 protected:
  virtual void setMotorVoltages_HAL(units::volt_t leftPower,
                                    units::volt_t rightPower) = 0;
  virtual frc::DifferentialDriveOdometry& getOdometry_HAL() = 0;
  virtual IGyro& getGyro_HAL() = 0;

  virtual void setMotorSpeeds_HAL(double leftPercent, double rightPercent) = 0;

  virtual double getLeftSpeedPercentage_HAL() = 0;
  virtual double getRightSpeedPercentage_HAL() = 0;

  virtual units::volt_t getLeftVoltage_HAL() = 0;
  virtual units::volt_t getRightVoltage_HAL() = 0;

  virtual TrivialEncoder& getLeftEncoder_HAL() = 0;
  virtual TrivialEncoder& getRightEncoder_HAL() = 0;

 private:
  static DeadBandEnforcer m_voltageDeadbandEnforcer;
  const frc::DifferentialDriveKinematics m_kinematics;
};