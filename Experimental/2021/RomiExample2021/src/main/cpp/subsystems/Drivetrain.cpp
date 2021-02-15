// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

#include "Constants.h"

#include <iostream>
#include <units/math.h>
#include <wpi/math>

using namespace DriveConstants;

// The Romi has the left and right motors set to
// PWM channels 0 and 1 respectively
// The Romi has onboard encoders that are hardcoded
// to use DIO pins 4/5 and 6/7 for the left and right
Drivetrain::Drivetrain() {
  m_leftEncoder.SetDistancePerPulse(
      wpi::math::pi * kWheelDiameter.to<double>() / kCountsPerRevolution);
  m_rightEncoder.SetDistancePerPulse(
      wpi::math::pi * kWheelDiameter.to<double>() / kCountsPerRevolution);
  ResetEncoders();
}

void Drivetrain::Periodic() {
  // This method will be called once per scheduler run.

  // constexpr bool reportFromGyro = false;
  // if (reportFromGyro)
  // {
  //   const double kP = 0.02;
  //   const double rawAngle = GetGyroAngleZ();
  //   const double clampedAngle = double(int(rawAngle) % 360);
  //   const double adjustedAngle = (clampedAngle <= 180)
  //                                    ? clampedAngle
  //                                    : clampedAngle - 360;
  //   const double error = -adjustedAngle; // our target angle is zero
  //   const double turnPower = kP * error;
  //   std::cout << "rawAngle: " << rawAngle
  //             << ", clamped: " << clampedAngle
  //             << ", adjusted: " << adjustedAngle
  //             << ", error: " << error
  //             << ", turnPower: " << turnPower
  //             << std::endl;
  // }
  // else
  // {
  //   // Need to convert distance travelled to degrees. The Standard Romi Chassis
  //   // found here https://www.pololu.com/category/203/romi-chassis-kits, has a
  //   // wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
  //   // or 5.551 inches. We then take into consideration the width of the tires.
  //   static auto inchPerDegree = (5.551_in * wpi::math::pi) / 360_deg;
  //   auto l = units::math::abs(GetLeftDistance());
  //   auto r = units::math::abs(GetRightDistance());
  //   auto distance = (l + r) / 2;
  //
  //   // Calculate the angle from the portion of the diameter the wheels have
  //   // covered.
  //   auto angle = distance / inchPerDegree;
  //   std::cout << "Distance: " << distance
  //             << ", angle: " << angle
  //             << std::endl;
  // }
}

void Drivetrain::ArcadeDrive(double xaxisSpeed, double zaxisRotate) {
  m_drive.ArcadeDrive(xaxisSpeed, zaxisRotate);
}

void Drivetrain::ResetEncoders() {
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();
}

int Drivetrain::GetLeftEncoderCount() {
  return m_leftEncoder.Get();
}

int Drivetrain::GetRightEncoderCount() {
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
