// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/XrpDriveBase.h"

#include "utils/DeadBandEnforcer.h"

namespace {
  constexpr double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0);  // 48.75:1
  constexpr double kCountsPerMotorShaftRev = 12.0;
  constexpr double kCountsPerRevolution =
      kCountsPerMotorShaftRev * kGearRatio;  // 585.0
  constexpr double kWheelDiameterMeters = 0.060;

  // Sample PID constants.
  constexpr double kP = 8.5;
  constexpr double kI = 0;
  constexpr double kD = 0;

  // Motor gains are for example purposes only, and must be determined for your
  // own robot.
  constexpr auto kS = 1_V;
  constexpr auto kV = 3 * (1_V * 1_s / 1_m);
  constexpr auto kA = 0 * (1_V * 1_s * 1_s / 1_m);
  constexpr auto kTrackWidth = 0.155_m;
}  // namespace

XrpDriveBase::XrpDriveBase()
    : IDrivebase(kTrackWidth, kP, kI, kD, kS, kV, kA),
      m_odometry{0_deg, 0_m, 0_m} {
  // Per docs: "The right motor will spin in a backward direction when positive
  // output is applied. Thus the corresponding motor controller needs to be
  // inverted in robot code."
  m_rightXrpMotor.SetInverted(true);
  m_leftXrpMotor.SetInverted(false);

  // Use meters as unit for encoder distances
  m_leftXrpEncoder.SetDistancePerPulse(
      (std::numbers::pi * kWheelDiameterMeters) / kCountsPerRevolution);
  m_rightXrpEncoder.SetDistancePerPulse(
      (std::numbers::pi * kWheelDiameterMeters) / kCountsPerRevolution);

  // Now that the encoders are configured, update the odometry to reflect
  // current data.
  updateOdometry();
}

void XrpDriveBase::setMotorVoltagesImpl(units::volt_t leftPower,
                                        units::volt_t rightPower) {
  m_leftXrpMotor.Set(convertVoltageToPercentSpeed(leftPower));
  m_rightXrpMotor.Set(convertVoltageToPercentSpeed(rightPower));
  m_leftXrpMotor.SetVoltage(leftPower);
  m_rightXrpMotor.SetVoltage(rightPower);
}
