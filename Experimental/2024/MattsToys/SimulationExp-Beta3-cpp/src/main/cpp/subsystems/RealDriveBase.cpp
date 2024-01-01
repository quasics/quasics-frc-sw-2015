// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RealDriveBase.h"

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
      m_odometry{m_trivialGyro->getRotation2d(),
                 m_leftTrivialEncoder->getPosition(),
                 m_rightTrivialEncoder->getPosition()} {
  SetName("RealDriveBase");

  // * Motor inversions
  m_leftSide.SetInverted(false);
  m_rightSide.SetInverted(true);

  // * RelativeEncoder configuration (to translate ticks to meters, etc.)
  configureEncoders();

  // Note that we aren't using a frc::DifferentialDrive object.  We're just
  // taking direct control of the two sides.
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

void RealDriveBase::setMotorVoltagesImpl(units::volt_t leftPower,
                                     units::volt_t rightPower) {
  m_leftSide.SetVoltage(leftPower);
  m_rightSide.SetVoltage(rightPower);
}
