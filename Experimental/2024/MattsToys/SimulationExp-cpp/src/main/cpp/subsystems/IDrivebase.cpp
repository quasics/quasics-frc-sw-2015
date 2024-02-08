// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IDrivebase.h"

#include "utils/DeadBandEnforcer.h"

DeadBandEnforcer IDrivebase::m_voltageDeadbandEnforcer(-0.001);
IDrivebase* IDrivebase::g_drivebaseSingleton = nullptr;

namespace {
  // Provides a deadband control for left/right wheel speeds.
  const DeadBandEnforcer kSpeedEnforcer{0.1};
}  // namespace

bool IDrivebase::m_logWheelSpeedData{false};

// This method will be called once per scheduler run
void IDrivebase::Periodic() {
  updateOdometry();
}

void IDrivebase::setMotorVoltages(units::volt_t leftVoltage,
                                  units::volt_t rightVoltage) {
  logValue("leftVolts", leftVoltage.value());
  logValue("rightVolts", rightVoltage.value());

  if (ENABLE_VOLTAGE_APPLICATION) {
    setMotorVoltages_HAL(leftVoltage, rightVoltage);
  }
}

/**
 * Controls the speeds for the drive base's left and right sides.  (This
 * includes calculating PID and feed-forward components.)
 *
 * @param speeds  desired wheel speeds for left/right side
 */
void IDrivebase::setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  setSpeedsImpl(speeds.left, speeds.right, true);
}

void IDrivebase::setSpeedsImpl(const units::meters_per_second_t leftSpeed,
                               const units::meters_per_second_t rightSpeed,
                               bool applyPid) {
  logValue("leftSpeed", leftSpeed.value());
  logValue("rightSpeed", rightSpeed.value());

  // A little "deadband enforcement".
  auto leftStabilized = kSpeedEnforcer(leftSpeed.value()) * 1_mps;
  auto rightStabilized = kSpeedEnforcer(rightSpeed.value()) * 1_mps;
  logValue("leftStable", leftStabilized.value());
  logValue("rightStable", rightStabilized.value());

  // Compute the basic voltages we'd need for the desired speeds.
  auto leftFeedforward = m_feedforward->Calculate(leftStabilized);
  auto rightFeedforward = m_feedforward->Calculate(rightStabilized);
  logValue("leftFF", leftFeedforward.value());
  logValue("rightFF", rightFeedforward.value());

  // Compute a delta, based on current/historical data.
  double leftPidOutput = applyPid
                             ? m_leftPIDController->Calculate(
                                   getLeftEncoder_HAL().getVelocity().value(),
                                   leftStabilized.value())
                             : 0;
  double rightPidOutput = applyPid
                              ? m_rightPIDController->Calculate(
                                    getRightEncoder_HAL().getVelocity().value(),
                                    rightStabilized.value())
                              : 0;
  logValue("leftPid", leftPidOutput);
  logValue("rightPid", rightPidOutput);

  // Sum the values to get left/right *actual* voltages needed.
  const auto leftVoltage = units::volt_t{leftPidOutput} + leftFeedforward;
  const auto rightVoltage = units::volt_t{rightPidOutput} + rightFeedforward;
  setMotorVoltages(leftVoltage, rightVoltage);
}
