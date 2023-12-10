// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IDrivebase.h"

// This method will be called once per scheduler run
void IDrivebase::Periodic() {
  updateOdometry();
}

/**
 * Controls the speeds for the drive base's left and right sides.  (This
 * includes calculating PID and feed-forward components.)
 *
 * @param speeds  desired wheel speeds for left/right side
 */
void IDrivebase::setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
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
  if (m_logWheelSpeedData) {
    frc::SmartDashboard::PutNumber("Left speed", speeds.left.value());
    frc::SmartDashboard::PutNumber("Right speed", speeds.right.value());
    frc::SmartDashboard::PutNumber("Left volts", leftVoltage.value());
    frc::SmartDashboard::PutNumber("Right volts", rightVoltage.value());
  }
}
