// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveBase.h"

#include <iostream>
#include <wpi/numbers>

DriveBase::DriveBase() {
  // Configure the motors.  (Must be done before adding them to motor
  // controller)
  leftFront.SetInverted(false);
  leftRear.SetInverted(false);
  rightFront.SetInverted(true);
  rightRear.SetInverted(true);

  leftSide.reset(new frc::MotorControllerGroup(leftFront, leftRear));
  rightSide.reset(new frc::MotorControllerGroup(rightFront, rightRear));

  drive.reset(new frc::DifferentialDrive(*leftSide, *rightSide));

  ////////////////////////////////////////
  // Configure the encoders.
  //
  // TODO: Check the math on all of this (at least empirically).

  // Default for the encoders is to report velocity in RPM, and distance as
  // fractions of a revolution; we want that to come back as m/s and meters,
  // respectively.

  // Compute the distance (in meters) for one revolution, first.
  const units::meter_t wheelCircumference =
      WHEEL_DIAMETER_INCHES *  // Will auto-convert to meters! :-)
      wpi::numbers::pi;

  // Conversion factor from rotations (or RPM) to meters (or m/s).
  const units::meter_t adjustmentForGearing =
      wheelCircumference / DRIVE_BASE_GEAR_RATIO;

  // Conversion factor from m/min to m/s
  const units::meter_t velocityAdjustment = adjustmentForGearing / 60;

  std::cout << "Wheel circumference (m): " << wheelCircumference.value()
            << std::endl;
  std::cout << "Adjustment for gearing: " << adjustmentForGearing.value()
            << std::endl;
  std::cout << "Velocity adj.: " << velocityAdjustment.value() << std::endl;

  leftFrontEncoder.SetVelocityConversionFactor(velocityAdjustment.value());
  leftRearEncoder.SetVelocityConversionFactor(velocityAdjustment.value());
  rightFrontEncoder.SetVelocityConversionFactor(velocityAdjustment.value());
  rightRearEncoder.SetVelocityConversionFactor(velocityAdjustment.value());

  leftFrontEncoder.SetPositionConversionFactor(adjustmentForGearing.value());
  leftRearEncoder.SetPositionConversionFactor(adjustmentForGearing.value());
  rightFrontEncoder.SetPositionConversionFactor(adjustmentForGearing.value());
  rightRearEncoder.SetPositionConversionFactor(adjustmentForGearing.value());

  ResetEncoders();
}

void DriveBase::ResetEncoders() {
  leftFrontEncoder.SetPosition(0);
  rightFrontEncoder.SetPosition(0);
  leftRearEncoder.SetPosition(0);
  rightRearEncoder.SetPosition(0);
}

void DriveBase::SetCoastingEnabled(bool tf) {
  const rev::CANSparkMax::IdleMode mode =
      tf ? rev::CANSparkMax::IdleMode::kCoast
         : rev::CANSparkMax::IdleMode::kBrake;
  leftFront.SetIdleMode(mode);
  rightFront.SetIdleMode(mode);
  leftRear.SetIdleMode(mode);
  rightRear.SetIdleMode(mode);
}

void DriveBase::TankDrive(double leftSpeed, double rightSpeed) {
  drive->TankDrive(leftSpeed, rightSpeed);
}

units::meters_per_second_t DriveBase::GetLeftSpeed() {
  return units::meters_per_second_t(leftRearEncoder.GetVelocity());
}

units::meters_per_second_t DriveBase::GetRightSpeed() {
  return units::meters_per_second_t(rightRearEncoder.GetVelocity());
}

units::meter_t DriveBase::GetLeftDistance() {
  return units::meter_t(leftRearEncoder.GetPosition());
}

units::meter_t DriveBase::GetRightDistance() {
  return units::meter_t(rightRearEncoder.GetPosition());
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {
}
