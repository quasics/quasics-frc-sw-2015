// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>

#include <numbers>

#include "sensors/IGyro.h"
#include "sensors/TrivialEncoder.h"

class IDrivebase {
 public:
  /** Maximum linear speed is 3 meters per second. */
  static constexpr double MAX_SPEED = 3.0;

  /** Maximum rotational speed is 1/2 rotation per second. */
  static constexpr double MAX_ANGULAR_SPEED = std::numbers::pi;

  using SimpleMotorFeedforward = frc::SimpleMotorFeedforward<units::meters>;
  using kv_unit = SimpleMotorFeedforward::kv_unit;
  using ka_unit = SimpleMotorFeedforward::ka_unit;

 private:
  const std::unique_ptr<frc::PIDController> m_leftPIDController;
  const std::unique_ptr<frc::PIDController> m_rightPIDController;
  const std::unique_ptr<SimpleMotorFeedforward> m_feedforward;
  const frc::DifferentialDriveKinematics m_kinematics;

 public:
  IDrivebase(units::meter_t trackWidth, double kP, double kI, double kD,
             units::volt_t kS, units::unit_t<kv_unit> kV)
      : IDrivebase(trackWidth, kP, kI, kD, kS, kV,
                   units::unit_t<ka_unit>(0.0)) {
  }
  IDrivebase(units::meter_t trackWidth, double kP, double kI, double kD,
             units::volt_t kS, units::unit_t<kv_unit> kV,
             units::unit_t<ka_unit> kA)
      : m_leftPIDController(new frc::PIDController(kP, kI, kD)),
        m_rightPIDController(new frc::PIDController(kP, kI, kD)),
        m_feedforward(
            new frc::SimpleMotorFeedforward<units::meters>(kS, kV, kA)),
        m_kinematics(trackWidth) {
  }
  virtual ~IDrivebase() = default;

  void arcadeDrive(double xPower, double rotationPower);

  void stop() {
    arcadeDrive(0, 0);
  }

  const frc::DifferentialDriveKinematics& getKinematics() {
    return m_kinematics;
  }

  const SimpleMotorFeedforward& getMotorFeedforward() {
    return *m_feedforward;
  }

  double getKP() {
    return m_leftPIDController->GetP();
  }

  double getKI() {
    return m_leftPIDController->GetI();
  }

  double getKD() {
    return m_leftPIDController->GetD();
  }

  /** @return current wheel speeds (in m/s) */
  frc::DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return frc::DifferentialDriveWheelSpeeds{getLeftEncoder().getVelocity(),
                                             getRightEncoder().getVelocity()};
  }

  /** Update the robot's odometry. */
  void updateOdometry() {
    getOdometry().Update(getGyro().getRotation2d(),
                         getLeftEncoder().getPosition(),
                         getRightEncoder().getPosition());
  }

  /** Check the current robot pose. */
  frc::Pose2d getPose() {
    return getOdometry().GetPose();
  }

  /** Resets robot odometry. */
  void resetOdometry(frc::Pose2d pose) {
    getLeftEncoder().reset();
    getRightEncoder().reset();
    getOdometry().ResetPosition(getGyro().getRotation2d(),
                                getLeftEncoder().getPosition(),
                                getRightEncoder().getPosition(), pose);
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  virtual void Periodic();

 protected:
  virtual void setMotorVoltages(double leftPower, double rightPower) = 0;

  virtual frc::DifferentialDriveOdometry& getOdometry() = 0;

  virtual TrivialEncoder& getLeftEncoder() = 0;

  virtual TrivialEncoder& getRightEncoder() = 0;

  virtual IGyro& getGyro() = 0;
};
