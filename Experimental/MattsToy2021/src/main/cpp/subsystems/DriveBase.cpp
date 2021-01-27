// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveBase.h"

#include <units/length.h>

#include <wpi/math>

#include "../../../../Common2021/EncoderHelpers.h"
#include "Constants.h"

/// Ticks per revolution on the Rev Neo motors.
constexpr double kTicksPerRevolution_NeoMotor = 42;

/// Gear ratio used for the 2020/2021 robots.
constexpr double kGearRatio_2020 = 10.71;

/// Wheel diameter on the 2020/2021 robots.
/// Makes use of the "units" library provided by WPI.  (See docs at
/// https://docs.wpilib.org/en/stable/docs/software/basic-programming/cpp-units.html.)
static constexpr units::length::inch_t kWheelDiameter = 6.0_in;

DriveBase::DriveBase()
    : leftFront(CANBusIds::SparkMax::Left_Front_No,
                rev::CANSparkMax::MotorType::kBrushless),
      leftRear(CANBusIds::SparkMax::Left_Rear_No,
               rev::CANSparkMax::MotorType::kBrushless),
      rightFront(CANBusIds::SparkMax::Right_Front_No,
                 rev::CANSparkMax::MotorType::kBrushless),
      rightRear(CANBusIds::SparkMax::Right_Rear_No,
                rev::CANSparkMax::MotorType::kBrushless) {
  SetSubsystem("Drivebase");
  leftMotors.SetInverted(true);
  SetCoastingEnabled(false);
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {
}

void DriveBase::SetCoastingEnabled(bool enabled) {
  rev::CANSparkMax::IdleMode mode =
      (enabled ? rev::CANSparkMax::IdleMode::kCoast
               : rev::CANSparkMax::IdleMode::kBrake);
  leftFront.SetIdleMode(mode);
  leftRear.SetIdleMode(mode);
  rightFront.SetIdleMode(mode);
  rightRear.SetIdleMode(mode);
}

void DriveBase::ArcadeDrive(double xaxisSpeed, double zaxisRotate,
                            double squareInputs) {
  drive.ArcadeDrive(xaxisSpeed, zaxisRotate, squareInputs);
}

void DriveBase::TankDrive(double leftSpeed, double rightSpeed) {
  drive.TankDrive(leftSpeed, rightSpeed);
}

void DriveBase::ResetEncoders() {
  leftFrontEncoder.SetPosition(0.0);
  leftRearEncoder.SetPosition(0.0);
  rightFrontEncoder.SetPosition(0.0);
  rightRearEncoder.SetPosition(0.0);
}

double DriveBase::GetLeftEncoderCount() {
  // For our encoders on Mae/Nike, it's the # of revolutions
  // TODO(mjh): Decide if we want to just use 1 encoder per side, or take an
  // average, or what.
  return leftFrontEncoder.GetPosition();
}

double DriveBase::GetRightEncoderCount() {
  // For our encoders on Mae/Nike, it's the # of revolutions
  return rightFrontEncoder.GetPosition();
}

units::meter_t DriveBase::GetRightDistance() {
  auto rightDistance = ((rightFrontEncoder.GetPosition()) / kGearRatio_2020) *
                       (kWheelDiameter * wpi::math::pi);
  return rightDistance;  // Units will automatically convert
}

units::meter_t DriveBase::GetLeftDistance() {
  auto leftDistance = ((leftFrontEncoder.GetPosition()) / kGearRatio_2020) *
                      (kWheelDiameter * wpi::math::pi);
  return leftDistance;  // Units will automatically convert
}
