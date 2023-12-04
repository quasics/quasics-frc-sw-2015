// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Subsystem.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#include "sensors/IGyro.h"
#include "sensors/TrivialEncoder.h"

/**
 * Constructor.
 *
 * @param trackWidthMeters  track width (from SysID using the "Drivetrain
 * (Angular)" test)
 * @param kP  kP value for PID control of motors
 * @param kI  kI value for PID control of motors
 * @param kD  kD value for PID control of motors
 * @param kS  voltage needed to overcome the drive motors' static friction
 * @param kV  voltage scaling value used to hold at a given velocity
 *
 * @see
 *     https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation
 * @see
 *     https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/identification-routine.html#track-width
 */
class IDrivebase {
 public:
  /** Maximum linear speed. */
  static constexpr units::meters_per_second_t MAX_SPEED{3.0};

  /** Maximum rotational speed is 1/2 rotation per second. */
  static constexpr units::radians_per_second_t MAX_ANGULAR_SPEED{
      std::numbers::pi};

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

  void arcadeDrive(units::meters_per_second_t xSpeed,
                   units::radians_per_second_t rot) {
    setSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
  }

  void stop() {
    arcadeDrive(0_mps, 0_rad_per_s);
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

  void setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
    // Compute the basic voltages we'd need for the desired speeds.
    auto leftFeedforward = m_feedforward->Calculate(speeds.left);
    auto rightFeedforward = m_feedforward->Calculate(speeds.right);

    // Compute a delta, based on current/historical data.
    double leftOutput = m_leftPIDController->Calculate(
        getLeftEncoder().getVelocity().value(), speeds.left.value());
    double rightOutput = m_rightPIDController->Calculate(
        getRightEncoder().getVelocity().value(), speeds.right.value());

    // Sum the values to get left/right *actual* voltages needed.
    const auto leftVoltage = units::volt_t{leftOutput} + leftFeedforward;
    const auto rightVoltage = units::volt_t{rightOutput} + rightFeedforward;
    setMotorVoltages(leftVoltage, rightVoltage);

    // Optional logging of speed data to SmartDashboard.
    constexpr bool LOG_SPEED_DATA = false;
    if (LOG_SPEED_DATA) {
      frc::SmartDashboard::PutNumber("Left speed", speeds.left.value());
      frc::SmartDashboard::PutNumber("Right speed", speeds.right.value());
      frc::SmartDashboard::PutNumber("Left volts", leftVoltage.value());
      frc::SmartDashboard::PutNumber("Right volts", rightVoltage.value());
    }
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
  virtual void resetOdometry(frc::Pose2d pose) {
    getLeftEncoder().reset();
    getRightEncoder().reset();
    getOdometry().ResetPosition(getGyro().getRotation2d(),
                                getLeftEncoder().getPosition(),
                                getRightEncoder().getPosition(), pose);
  }

  frc2::Subsystem& asFrcSubsystem() {
    // Note that I need to do the "dynamic_cast" below in order to safely
    // convert types between the custom "IDrivebase" interface that the smart
    // pointer knows about and a "Subsystem" type that exposes the
    // "SetDefaultCommand" method that we want to use.  (In Java, this
    // hoop-jumping isn't needed.)
    return dynamic_cast<frc2::Subsystem&>(*this);
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  virtual void Periodic();

 protected:
  virtual void setMotorVoltages(units::volt_t leftPower,
                                units::volt_t rightPower) = 0;

  virtual frc::DifferentialDriveOdometry& getOdometry() = 0;

  virtual TrivialEncoder& getLeftEncoder() = 0;

  virtual TrivialEncoder& getRightEncoder() = 0;

  virtual IGyro& getGyro() = 0;
};
