// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

#include <wpi/math>

#include "../../../../../Common2021/LoggingHelpers.h"
#include "Constants.h"

using namespace DriveConstants;

// The Romi has the left and right motors set to
// PWM channels 0 and 1 respectively
// The Romi has onboard encoders that are hardcoded
// to use DIO pins 4/5 and 6/7 for the left and right
Drivetrain::Drivetrain() : m_odometry(units::degree_t(m_gyro.GetAngleZ())) {
  // Set the encoders up so that when we call "GetRate()", we'll get the
  // rate of turn in meters/sec.
  m_leftEncoder.SetDistancePerPulse(
      wpi::math::pi * kWheelDiameter.to<double>() / kCountsPerRevolution);
  m_rightEncoder.SetDistancePerPulse(
      wpi::math::pi * kWheelDiameter.to<double>() / kCountsPerRevolution);

  ResetEncoders();
}

// This method will be called once per scheduler run.
void Drivetrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
  auto rotation = GetZAxisGyro().GetRotation2d();
  auto leftDistance = units::meter_t(m_leftEncoder.GetDistance());
  auto rightDistance = units::meter_t(m_rightEncoder.GetDistance());

  LOG_EVERY_N_TIMES(10, std::cout << "Odometry: " << rotation.Degrees() << " / "
                                  << leftDistance << " / " << rightDistance
                                  << std::endl;)

  m_odometry.Update(rotation, leftDistance, rightDistance);
}

void Drivetrain::ArcadeDrive(double xaxisSpeed, double zaxisRotate,
                             bool squareInputs) {
  m_drive.ArcadeDrive(xaxisSpeed, zaxisRotate, squareInputs);
}

void Drivetrain::TankDrive(double leftSpeed, double rightSpeed) {
  m_drive.TankDrive(leftSpeed, rightSpeed);
}

void Drivetrain::ResetEncoders() {
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();
}

double Drivetrain::GetLeftEncoderCount() {
  return m_leftEncoder.Get();
}

double Drivetrain::GetRightEncoderCount() {
  return m_rightEncoder.Get();
}

units::meter_t Drivetrain::GetLeftDistance() {
  return units::meter_t(m_leftEncoder.GetDistance());
}

units::meter_t Drivetrain::GetRightDistance() {
  return units::meter_t(m_rightEncoder.GetDistance());
}

units::meter_t Drivetrain::GetAverageDistance() {
  return (GetLeftDistance() + GetRightDistance()) / 2.0;
}

double Drivetrain::GetAccelX() {
  return m_accelerometer.GetX();
}

double Drivetrain::GetAccelY() {
  return m_accelerometer.GetY();
}

double Drivetrain::GetAccelZ() {
  return m_accelerometer.GetZ();
}

double Drivetrain::GetGyroAngleX() {
  return m_gyro.GetAngleX();
}

double Drivetrain::GetGyroAngleY() {
  return m_gyro.GetAngleY();
}

double Drivetrain::GetGyroAngleZ() {
  return m_gyro.GetAngleZ();
}

void Drivetrain::ResetGyro() {
  m_gyro.Reset();
}

frc::Pose2d Drivetrain::GetPose() {
  return m_odometry.GetPose();
}

/**
 * Returns the current wheel speeds of the robot.
 *
 * @return The current wheel speeds.
 */
frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds() {
  return {units::meters_per_second_t(m_leftEncoder.GetRate()),
          units::meters_per_second_t(m_rightEncoder.GetRate())};
}

/**
 * Resets the odometry to the specified pose.
 *
 * @param pose The pose to which to set the odometry.
 */
void Drivetrain::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, GetZAxisGyro().GetRotation2d());
}

/**
 * Controls each side of the drive directly with a voltage.
 *
 * @param left the commanded left output
 * @param right the commanded right output
 */
void Drivetrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotor.SetVoltage(left);
  m_rightMotor.SetVoltage(-right);
  m_drive.Feed();
}
