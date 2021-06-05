// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

#include <frc/shuffleboard/Shuffleboard.h>

#include <wpi/math>

#include "../../../../../Common2021/LoggingHelpers.h"
#include "Constants.h"

using namespace DriveConstants;

/// The tab used for commands/debugging widgets that are specific to this
/// subsystem.
static const char* const kShuffleboardTabName = "DriveBase";

// The Romi has the left and right motors set to
// PWM channels 0 and 1 respectively
// The Romi has onboard encoders that are hardcoded
// to use DIO pins 4/5 and 6/7 for the left and right
Drivetrain::Drivetrain() : m_odometry(units::degree_t(0)) {
  // Set the encoders up so that when we call "GetRate()", we'll get the
  // rate of turn in meters/sec.
  m_leftEncoder.SetDistancePerPulse(
      wpi::math::pi * kWheelDiameter.to<double>() / kCountsPerRevolution);
  m_rightEncoder.SetDistancePerPulse(
      wpi::math::pi * kWheelDiameter.to<double>() / kCountsPerRevolution);

  ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));

  ConfigureShuffleboard();
}

// Note: at start, values *should* all be 0, but they'll be updated as soon as
// Periodic() is invoked, anyway.
void Drivetrain::ConfigureShuffleboard() {
  auto& tab = frc::Shuffleboard::GetTab(kShuffleboardTabName);
  leftDistance = tab.AddPersistent("L Distance", 0).GetEntry();
  rightDistance = tab.AddPersistent("R Distance", 0).GetEntry();
  leftSpeed = tab.AddPersistent("L Speed", 0).GetEntry();
  rightSpeed = tab.AddPersistent("R Speed", 0).GetEntry();
  rotation = tab.AddPersistent("Rotation", 0).GetEntry();
}

void Drivetrain::AddToShuffleboard(wpi::StringRef label, frc::Sendable* data) {
  if (data != nullptr) {
    auto& tab = frc::Shuffleboard::GetTab(kShuffleboardTabName);
    tab.Add(label, *data);
  }
}

void Drivetrain::UpdateShuffleboard() {
  const auto wheelSpeeds = GetWheelSpeeds();

  leftDistance.SetDouble(GetLeftDistance().to<double>());
  rightDistance.SetDouble(GetRightDistance().to<double>());
  leftSpeed.SetDouble(wheelSpeeds.left.to<double>());
  rightSpeed.SetDouble(wheelSpeeds.right.to<double>());
  rotation.SetDouble(m_odometry.GetPose().Rotation().Degrees().to<double>());
}

// This method will be called once per scheduler run.
void Drivetrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
  auto rotation = GetZAxisGyro().GetRotation2d();
  auto leftDistance = units::meter_t(m_leftEncoder.GetDistance());
  auto rightDistance = units::meter_t(m_rightEncoder.GetDistance());

  m_odometry.Update(rotation, leftDistance, rightDistance);

  UpdateShuffleboard();

  // Every so often, report on current odometry.
  LOG_EVERY_N_SECONDS(10, std::cout << "Odometry: " << rotation.Degrees()
                                    << " / " << leftDistance << " / "
                                    << rightDistance << std::endl;)
}

void Drivetrain::ArcadeDrive(double xaxisSpeed, double zaxisRotate,
                             bool squareInputs) {
  m_drive.ArcadeDrive(xaxisSpeed, zaxisRotate, squareInputs);
}

void Drivetrain::CurvatureDrive(double xaxisSpeed, double zaxisRotate,
                                bool isQuickTurn) {
  m_drive.CurvatureDrive(xaxisSpeed, zaxisRotate, isQuickTurn);
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
