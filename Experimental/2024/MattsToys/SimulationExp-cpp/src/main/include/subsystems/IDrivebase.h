// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/RobotController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Subsystem.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>
#include <stdexcept>

#include "sensors/IGyro.h"
#include "sensors/TrivialEncoder.h"
#include "utils/DeadBandEnforcer.h"

/**
 * This class provides the framework for a common interface to be used in
 * controlling a drive base (real or simulated).
 *
 * The goal is to enforce a separation (sometimes referred to as a "hardware
 * abstraction layer") between functions that deal directly with the hardware
 * (e.g., configuring motors, setting voltages to be passed through to them,
 * etc.), and functions that *use* those hardware-specific pieces to get more
 * general functionality done.
 *
 * For example, implementing "arcade" or "tank" drive is pretty much the same
 * for any sort of a differential drive base when dealing with things like
 * speed constraints and deadband evaluation, up to the point when you actually
 * need to say, "OK, make _these_ motors run at speed X, or get sent voltage Y".
 *
 * In this class, we:
 *   - Define a "contract" that must be implemented by further subclasses,
 *     which will handle the task of configuring/interacting with the real
 *     hardware (e.g., an FRC drive base, or an XRP "toy bot", etc.), in the
 *     form of abstract functions (e.g., "virtual void
 *     setMotorVoltages_HAL(double left, double right) = 0;") that hide the
 *     details associated with any specific hardware.
 *   - Define and implement the "hardware-agnostic" functionality that is built
 *     using/on top of that contract (e.g., arcadeDrive(), tankDrive(), etc.).
 *
 * TODO: Move the odometry setup and direct usage into this class, rather than
 * pushing it down to the derived classes.
 */
class IDrivebase : public frc2::SubsystemBase {
  // Useful class constants.
 public:
  static const std::string_view BULLETIN_BOARD_POSE_KEY;
  static const std::string_view BULLETIN_BOARD_DIRECTION_KEY;
  static const std::string_view BULLETIN_BOARD_DIRECTION_FORWARD_VALUE;
  static const std::string_view BULLETIN_BOARD_DIRECTION_REVERSE_VALUE;
  static const std::string_view BULLETIN_BOARD_DIRECTION_TURNING_VALUE;
  static const std::string_view BULLETIN_BOARD_DIRECTION_STOPPED_VALUE;

  /** Maximum linear speed (@ 100% of rated speed). */
  static constexpr units::meters_per_second_t MAX_SPEED{3.0};

  /** Maximum rotational speed is 1/2 rotation per second. */
  static constexpr units::radians_per_second_t MAX_ANGULAR_SPEED{
      std::numbers::pi};

  /** Used to disable actual motor control (if neeed during debugging). */
  static constexpr bool ENABLE_VOLTAGE_APPLICATION = true;

  // Convenient type aliases, letting us just say things like "ka_unit" inside
  // this class's code, instead of having to use the whole fully-qualified name
  // every time.
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
  static bool m_logWheelSpeedData;

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
    // To paraphrase Connor and Duncan MacLeod, "There should be only one!"
    if (g_drivebaseSingleton != nullptr) {
      throw std::logic_error("Drivebase is supposed to be a singleton");
    }
    g_drivebaseSingleton = this;
  }

  /**
   * Convenience function to get access to the singleton drivebase.  This should
   * only be used to access data (e.g., current position), and not to manipulate
   * the drive base from commands (since that won't allow for subsystem
   * dependencies).
   */
  static IDrivebase& GetDrivebase() {
    if (g_drivebaseSingleton == nullptr) {
      throw std::logic_error("Drivebase isn't set up yet");
    }
    return *g_drivebaseSingleton;
  }

  /**
   * Destructor.  (Must be present and virtual in order for derived classes to
   * be cleaned up correctly.)
   */
  virtual ~IDrivebase() {
    if (g_drivebaseSingleton == this) {
      // Clear the pointer to singleton
      g_drivebaseSingleton = nullptr;
    }
  }

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
    logValue("xSpeed", xSpeed.value());
    logValue("rotSpeed", rot.value());

    // Convert the requested speeds to left/right velocities.
    frc::ChassisSpeeds speeds;
    speeds.vx = xSpeed;
    speeds.omega = rot;
    const auto wheelSpeeds = m_kinematics.ToWheelSpeeds(speeds);

    // Set left/right speeds.
    setSpeeds(wheelSpeeds);
  }

  /** Helper method to stop the robot. */
  void stop() {
    setSpeedsImpl(0_mps, 0_mps, false);
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
    return frc::DifferentialDriveWheelSpeeds{
        getLeftEncoder_HAL().getVelocity(),
        getRightEncoder_HAL().getVelocity()};
  }

  /** Update the robot's odometry. */
  void updateOdometry() {
    getOdometry_HAL().Update(getGyro_HAL().getRotation2d(),
                             getLeftEncoder_HAL().getPosition(),
                             getRightEncoder_HAL().getPosition());
  }

  /** Check the current robot pose. */
  frc::Pose2d getPose() {
    return getOdometry_HAL().GetPose();
  }

  /** Resets robot odometry. */
  virtual void resetOdometry(frc::Pose2d pose) {
    getLeftEncoder_HAL().reset();
    getRightEncoder_HAL().reset();
    getOdometry_HAL().ResetPosition(getGyro_HAL().getRotation2d(),
                                    getLeftEncoder_HAL().getPosition(),
                                    getRightEncoder_HAL().getPosition(), pose);
  }

  // Functions to support logging data used for driving.
 public:
  /**
   * Enables/disables logging of (target) wheel speeds and voltages to the
   * SmartDashboard.
   * @see #isLoggingEnabled
   */
  static void enableLogging(bool tf) {
    m_logWheelSpeedData = tf;
  }

  /**
   * @return true iff logging is enabled.
   * @see #enableLogging(bool)
   */
  static bool isLoggingEnabled() {
    return m_logWheelSpeedData;
  }

  /**
   * Logs the specified label/value to the SmartDashboard (if enabled).
   * @see #isLoggingEnabled
   */
  static void logValue(std::string_view label, double val) {
    if (m_logWheelSpeedData) {
      frc::SmartDashboard::PutNumber(label, val);
    }
  }

  // Standard WPILib functions, which we're going to override.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  virtual void Periodic();

  // Supporting SysId profiling
 private:
  using SysIdRoutine = frc2::sysid::SysIdRoutine;
  using SysIdConfig = frc2::sysid::Config;
  using SysIdMechanism = frc2::sysid::Mechanism;

  SysIdRoutine m_sysIdRoutine{
      // Tell it to use the default configurations for setting up the tests.
      SysIdConfig{
          /*rampRate (defaults to 1v/s)*/ {},
          /*stepVoltage (defaults to 7v)*/ {},
          /*timeout (defaults to 10sec)*/ {},
          /*recordState (defaults to using "stock" WPILib logging)*/ {}},

      // Provide the necessar "mechanisms" for handling SysId profile data
      // generation.
      SysIdMechanism{
          // This is a function (written as a lamba) to use in
          // controlling the motors directly, by means of adjusting
          // the voltages sent to them.
          [this](units::volt_t volts) {
            this->setMotorVoltages_HAL(volts, volts);
          },
          // This is a function for recording data "frames" (i.e.,
          // snapshots of the current state, such as during the power
          // ramp-up process) for the left- and right-side motors. If
          // we have more than one motor on a side for some
          // subclass, we'll assume that the sublcass providing us with an
          // appropriate value (e.g., either based on one motor from that side,
          // or an average across the motors, etc.).
          [this](frc::sysid::SysIdRoutineLog* log) {
            // Give the provided "log" object information about the status of
            // the left-side motor(s).
            log->Motor("drive-left")
                .voltage(getLeftSpeedPercentage_HAL() *
                         frc::RobotController::GetBatteryVoltage())
                .position(getLeftEncoder_HAL().getPosition())
                .velocity(getLeftEncoder_HAL().getVelocity());
            // Give the provided "log" object information about the status of
            // the right-side motor(s).
            log->Motor("drive-right")
                .voltage(getRightSpeedPercentage_HAL() *
                         frc::RobotController::GetBatteryVoltage())
                .position(getRightEncoder_HAL().getPosition())
                .velocity(getRightEncoder_HAL().getVelocity());
          },
          // The subsystem being profiled (i.e., a pointer to the drive base),
          // so that the commands being generated can specify a dependency on
          // it.
          this}};

 public:
  /**
   * @return a CommandPtr for use in running quasistatic profiling in the
   * specified direction.
   */
  frc2::CommandPtr sysIdQuasistatic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Quasistatic(direction);
  }

  /**
   * @return a CommandPtr for use in running dynamic profiling in the
   * specified direction.
   */
  frc2::CommandPtr sysIdDynamic(frc2::sysid::Direction direction) {
    return m_sysIdRoutine.Dynamic(direction);
  }

  // Internal helper methods.
 protected:
  /**
   * Sets the voltages for the motors on the left and right sides, and
   * "remembers" the settings, so that they can be reapplied regularly.
   *
   * @see #Periodic()
   * @see #setMotorVoltages_HAL(units::volt_t, units::volt_t)
   */
  void setMotorVoltages(units::volt_t leftPower, units::volt_t rightPower);

  void setSpeedsImpl(const units::meters_per_second_t leftSpeed,
                     const units::meters_per_second_t rightSpeed,
                     bool applyPid);

  static double convertVoltageToPercentSpeed(units::volt_t volts) {
    const double inputVoltage = frc::RobotController::GetInputVoltage();
    const double mps = (volts.value() / inputVoltage);
    const double speedPercentage =
        m_voltageDeadbandEnforcer(mps / MAX_SPEED.value());
    return speedPercentage;
  }

  // The "hardware abstraction layer" methods.  These are functions that
  // *must* be overridden by the derived classes, providing access to the
  // underlying hardware to support the common functionality defined
  // above.
 protected:
  /** Actually sets the voltages for the motors on the left and right sides. */
  virtual void setMotorVoltages_HAL(units::volt_t leftPower,
                                    units::volt_t rightPower) = 0;

  /** @return the odometry tracker for the underlying drive base. */
  virtual frc::DifferentialDriveOdometry& getOdometry_HAL() = 0;

  /** @return reference to a TrivialEncoder for the left side of the robot. */
  virtual TrivialEncoder& getLeftEncoder_HAL() = 0;

  /** @return reference to a TrivialEncoder for the right side of the robot. */
  virtual TrivialEncoder& getRightEncoder_HAL() = 0;

  /**
   * @return reference to an IGyro that can be used to determine the robot's
   * heading.
   */
  virtual IGyro& getGyro_HAL() = 0;

  /**
   * @return the most recent "power %" setting for the left-side motor(s)
   * (possibly back-calculated from voltages)
   */
  virtual double getLeftSpeedPercentage_HAL() = 0;

  /**
   * @return the most recent "power %" setting for the right-side motor(s)
   * (possibly back-calculated from voltages)
   */
  virtual double getRightSpeedPercentage_HAL() = 0;

 private:
  static DeadBandEnforcer m_voltageDeadbandEnforcer;

  static IDrivebase* g_drivebaseSingleton;
};
