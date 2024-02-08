// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RealDriveBase.h"

using IdleMode = rev::CANSparkMax::IdleMode;
using std::numbers::pi;

////////////////////////////////////////////////
/// From 2023 "ChargedUp" constants for Sally
constexpr auto kS = 0.19529_V;
constexpr auto kV = 2.2329 * (1_V * 1_s / 1_m);
constexpr auto kA = 0 * (1_V * 1_s * 1_s / 1_m);
constexpr double kP = 0.29613;
constexpr double kI = 0;
constexpr double kD = 0;
constexpr units::length::inch_t TRACK_WIDTH_METERS_SALLY = 22.0_in;

namespace RobotPhysics {
  /** Drive base gear ratio used for Mae (2020/2021 robot). */
  constexpr double DRIVEBASE_GEAR_RATIO_MAE = 10.71;
  /** Drive base gear ratio used for Sally (2022 robot/development drive base).
   */
  constexpr double DRIVEBASE_GEAR_RATIO_SALLY = 8.45;
  /** Drive base gear ratio used for Gladys (2023 robot). */
  constexpr double DRIVEBASE_GEAR_RATIO_GLADYS = 8.45;

  /**
   * Drive base gear ratio on the currently-targeted robot.
   *
   * Used to isolate the gear ratio for the robot we're actually building for,
   * from the various "known values" for each of the possible robots.
   */
  constexpr double DRIVEBASE_GEAR_RATIO = DRIVEBASE_GEAR_RATIO_SALLY;

  /** Wheel diameter used with all of our 2020-2023 robot drive bases. */
  constexpr units::length::inch_t WHEEL_DIAMETER = 6.0_in;
}  // namespace RobotPhysics

RealDriveBase::RealDriveBase()
    : IDrivebase(TRACK_WIDTH_METERS_SALLY, kP, kI, kD, kS, kV, kA),
      // Initial (dummy) configuration of the odometry, to be updated once the
      // encoders are configured.
      m_odometry{0_deg, 0_m, 0_m} {
  SetName("RealDriveBase");

  // * Motor configuration
  configureMotors();

  // * RelativeEncoder configuration (to translate ticks to meters, etc.)
  configureEncoders();

  // Update the odometry, now that the encoders have been configured.
  updateOdometry();

  // Note that we aren't using a frc::DifferentialDrive object.  We're just
  // taking direct control of the two sides.
}

void RealDriveBase::enableCoastingMode(bool tf) {
  const auto mode = (tf ? IdleMode::kCoast : IdleMode::kBrake);
#if defined(ENABLE_DRIVE_BASE_LEAD_FOLLOW)
  m_leftLeader.SetIdleMode(mode);
  m_rightLeader.SetIdleMode(mode);
#else
  m_leftFront.SetIdleMode(mode);
  m_leftBack.SetIdleMode(mode);
  m_rightFront.SetIdleMode(mode);
  m_rightBack.SetIdleMode(mode);
#endif
}

void RealDriveBase::configureMotors() {
  // * Motor inversions (if needed, and the motors aren't already
  // soft-configured).
  //
  // m_leftLeader.SetInverted(false);
  // m_rightLeader.SetInverted(true);

  // Configure initial coast/brake mode
  enableCoastingMode(false);
}

void RealDriveBase::configureEncoders() {
  // Calculate wheel circumference (distance travelled per wheel revolution).
  const units::meter_t wheelCircumference = RobotPhysics::WHEEL_DIAMETER * pi;

  // Compute distance traveled per rotation of the motor.
  const units::meter_t gearingConversion =
      wheelCircumference / RobotPhysics::DRIVEBASE_GEAR_RATIO;

  // Compute conversion factor (used to change "(motor) RPM" to "m/sec").
  const units::meter_t velocityCorrection = gearingConversion / 60;

  // Update encoders so that they will report distance as meters traveled,
  // rather than rotations.
  m_leftFrontEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_leftBackEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_rightFrontEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_rightBackEncoder.SetPositionConversionFactor(gearingConversion.value());

  // Update encoders so that they will report velocity as m/sec, rather than
  // RPM.
  m_leftFrontEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_leftBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_rightFrontEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_rightBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());

  resetEncoders();
}

void RealDriveBase::setMotorVoltages_HAL(units::volt_t leftPower,
                                         units::volt_t rightPower) {
#if defined(ENABLE_DRIVE_BASE_LEAD_FOLLOW)
  m_leftLeader.SetVoltage(leftPower);
  m_rightLeader.SetVoltage(rightPower);
#else
  m_leftBack.SetVoltage(leftPower);
  m_rightBack.SetVoltage(rightPower);
  m_leftFront.SetVoltage(leftPower);
  m_rightFront.SetVoltage(rightPower);
#endif
}
