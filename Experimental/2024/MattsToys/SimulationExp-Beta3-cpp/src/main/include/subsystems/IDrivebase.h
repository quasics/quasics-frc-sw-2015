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
 * This class provides the framework for a common interface to be used in
 * controlling a drive base (real or simulated), and is intended to be a
 * "mix-in" base class for derived types, along with the actual SubsystemBase
 * class.
 *
 * For example:
 * <code>
 *   class ActualDrivebase : public frc2::SubsystemBase, public IDrivebase {
 *     . . .
 *     ActualDriveBase::ActualDriveBase()
 *       : IDrivebase(<constants from SysID...>) { ... }
 *     virtual Periodic() {
 *       IDrivebase::Periodic();  // See comments below.
 *     }
 *     . . .
 *   };
 * </code>
 */
class IDrivebase {
  // Useful class constants.
 public:
  /** Maximum linear speed (@ 100% of rated speed). */
  static constexpr units::meters_per_second_t MAX_SPEED{3.0};

  /** Maximum rotational speed is 1/2 rotation per second. */
  static constexpr units::radians_per_second_t MAX_ANGULAR_SPEED{
      std::numbers::pi};

  // Convenient type aliases.
 public:
  using SimpleMotorFeedforward = frc::SimpleMotorFeedforward<units::meters>;
  using kv_unit = SimpleMotorFeedforward::kv_unit;
  using ka_unit = SimpleMotorFeedforward::ka_unit;

  // Core underlying data members, which will be initialized using values
  // provided by the derived classes, since things like PID constants, etc.,
  // will be specific to the actual hardware (simulated or real).
 private:
  /** PID controller for motors on the left side of the drive base. */
  const std::unique_ptr<frc::PIDController> m_leftPIDController;
  /** PID controller for motors on the right side of the drive base. */
  const std::unique_ptr<frc::PIDController> m_rightPIDController;
  /**
   * Feed-forward control for drive motors.  (Like other resources in WPILib,
   * it is assumed that the motors on the left and the right side are consistent
   * with each other, and thus a single feed-forward calculator can be used for
   * both sides.)
   */
  const std::unique_ptr<SimpleMotorFeedforward> m_feedforward;
  /**
   * Kinematics charactistics for the underlying drive base.
   */
  const frc::DifferentialDriveKinematics m_kinematics;

 private:
  /** Iff true, enables logging wheel speeds/voltages to SmartDashboard. */
  bool m_logWheelSpeedData = false;

 public:
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
  IDrivebase(units::meter_t trackWidth, double kP, double kI, double kD,
             units::volt_t kS, units::unit_t<kv_unit> kV)
      : IDrivebase(trackWidth, kP, kI, kD, kS, kV,
                   units::unit_t<ka_unit>(0.0)) {
  }

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
   * @param kA  voltage scaling value used to hold at a given acceleration
   *
   * @see
   *     https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation
   * @see
   *     https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/identification-routine.html#track-width
   */
  IDrivebase(units::meter_t trackWidth, double kP, double kI, double kD,
             units::volt_t kS, units::unit_t<kv_unit> kV,
             units::unit_t<ka_unit> kA)
      : m_leftPIDController(new frc::PIDController(kP, kI, kD)),
        m_rightPIDController(new frc::PIDController(kP, kI, kD)),
        m_feedforward(
            new frc::SimpleMotorFeedforward<units::meters>(kS, kV, kA)),
        m_kinematics(trackWidth) {
  }

  /**
   * Destructor.  (Must be present and virtual in order for derived classes to
   * be cleaned up correctly.)
   */
  virtual ~IDrivebase() = default;

  /**
   * Basic arcade drive control.
   *
   * @param xSpeed  speed along the X axis (positive == forward, negative ==
   * backward)
   * @param rot     rotational speed
   *
   * @see #setSpeeds
   * @see #setMotorVoltages
   */
  void arcadeDrive(units::meters_per_second_t xSpeed,
                   units::radians_per_second_t rot) {
    setSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
  }

  /** Helper method to stop the robot. */
  void stop() {
    arcadeDrive(0_mps, 0_rad_per_s);
  }

  /** @return the kinematics data for the drive base. */
  const frc::DifferentialDriveKinematics& getKinematics() {
    return m_kinematics;
  }

  /** @return the feed-forward calculator for the drive base. */
  const SimpleMotorFeedforward& getMotorFeedforward() {
    return *m_feedforward;
  }

  /**
   * @return the kP value for the drive base (as provided by the derived
   * classes at construction).
   */
  double getKP() {
    return m_leftPIDController->GetP();
  }

  /**
   * @return the kI value for the drive base (as provided by the derived
   * classes at construction).
   */
  double getKI() {
    return m_leftPIDController->GetI();
  }

  /**
   * @return the kD value for the drive base (as provided by the derived
   * classes at construction).
   */
  double getKD() {
    return m_leftPIDController->GetD();
  }

  /**
   * Controls the speeds for the drive base's left and right sides.  (This
   * includes calculating PID and feed-forward components.)
   *
   * @param speeds  desired wheel speeds for left/right side
   */
  void setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);

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

  /**
   * @return a reference to the underlying IDrivebase as an frc2::Subsystem
   * object (if the underlying derived class has been appropriately derived from
   * that type, as well as this one).
   */
  frc2::Subsystem& asFrcSubsystem() {
    // Note that I need to do the "dynamic_cast" below in order to safely
    // convert types between the custom "IDrivebase" interface that the smart
    // pointer knows about and a "Subsystem" type that exposes the
    // "SetDefaultCommand" method that we want to use.  (In Java, this
    // hoop-jumping isn't needed.)
    return dynamic_cast<frc2::Subsystem&>(*this);
  }

  /**
   * Enables/disables logging of (target) wheel speeds and voltages to the
   * SmartDashboard.
   */
  void enableLogging(bool tf) {
    m_logWheelSpeedData = tf;
  }

  // Standard WPILib functions, which we're going to override.  (Note that
  // because we're using "multiple inheritance", the derived classes will need
  // to have at least a trivial version of these functions that explicitly
  // invokes this version, or else the compiler will complain that it doesn't
  // know how to deal with them.)
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  virtual void Periodic();

  // Methods that *must* be overridden by the derived classes, providing access
  // to the underlying hardware to support the common functionality defined
  // above.
 protected:
  /** Sets the voltages for the motors on the left and right sides. */
  virtual void setMotorVoltages(units::volt_t leftPower,
                                units::volt_t rightPower) = 0;

  /** @return the odometry tracker for the underlying drive base. */
  virtual frc::DifferentialDriveOdometry& getOdometry() = 0;

  /** @return reference to a TrivialEncoder for the left side of the robot. */
  virtual TrivialEncoder& getLeftEncoder() = 0;

  /** @return reference to a TrivialEncoder for the right side of the robot. */
  virtual TrivialEncoder& getRightEncoder() = 0;

  /**
   * @return reference to an IGyro that can be used to determine the robot's
   * heading.
   */
  virtual IGyro& getGyro() = 0;
};