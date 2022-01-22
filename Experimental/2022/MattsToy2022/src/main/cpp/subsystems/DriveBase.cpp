// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveBase.h"

#include <iostream>
#include <wpi/numbers>

DriveBase::DriveBase() {
  // Configure the motors.  (Must be done before adding them to motor
  // controller groups.)
  m_leftFront.SetInverted(false);
  m_leftRear.SetInverted(false);
  m_rightFront.SetInverted(true);
  m_rightRear.SetInverted(true);

  // Build the motor controller groups and differential drive.
  m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftRear));
  m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightRear));

  m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));

  // Set up the encoders so that they report units that we care about.
  ConfigureEncoders();
}

/**
 * Configures the encoders so that they will report back speeds and positions in
 * units that we generally care about (e.g., m/s and meters, rather than RPM and
 * fractions of a rotation).
 */
void DriveBase::ConfigureEncoders() {
  // TODO: Check the math on all of this (at least empirically).

  // Compute the distance (in meters) for one rotation of the wheel, first.
  const units::meter_t wheelCircumference =
      WHEEL_DIAMETER_INCHES *  // Will auto-convert to meters! :-)
      wpi::numbers::pi;
  std::cout << "Wheel circumference (m): " << wheelCircumference.value()
            << std::endl;

  // Conversion factor from units in rotations (or RPM) to meters (or m/s).
  const units::meter_t adjustmentForGearing =
      wheelCircumference / DRIVE_BASE_GEAR_RATIO;
  std::cout << "Adjustment for gearing (m/rotation): "
            << adjustmentForGearing.value() << std::endl;

  m_leftFrontEncoder.SetPositionConversionFactor(adjustmentForGearing.value());
  m_leftRearEncoder.SetPositionConversionFactor(adjustmentForGearing.value());
  m_rightFrontEncoder.SetPositionConversionFactor(adjustmentForGearing.value());
  m_rightRearEncoder.SetPositionConversionFactor(adjustmentForGearing.value());

  // Further conversion factor from m/min to m/s (used for velocity).
  const units::meter_t velocityAdjustment = adjustmentForGearing / 60;
  std::cout << "Velocity adj.: " << velocityAdjustment.value() << std::endl;

  m_leftFrontEncoder.SetVelocityConversionFactor(velocityAdjustment.value());
  m_leftRearEncoder.SetVelocityConversionFactor(velocityAdjustment.value());
  m_rightFrontEncoder.SetVelocityConversionFactor(velocityAdjustment.value());
  m_rightRearEncoder.SetVelocityConversionFactor(velocityAdjustment.value());

  ResetEncoders();
}

void DriveBase::ResetEncoders() {
  m_leftFrontEncoder.SetPosition(0);
  m_rightFrontEncoder.SetPosition(0);
  m_leftRearEncoder.SetPosition(0);
  m_rightRearEncoder.SetPosition(0);
}

void DriveBase::SetCoastingEnabled(bool tf) {
  const rev::CANSparkMax::IdleMode mode =
      tf ? rev::CANSparkMax::IdleMode::kCoast
         : rev::CANSparkMax::IdleMode::kBrake;
  m_leftFront.SetIdleMode(mode);
  m_rightFront.SetIdleMode(mode);
  m_leftRear.SetIdleMode(mode);
  m_rightRear.SetIdleMode(mode);
}

void DriveBase::TankDrive(double leftSpeed, double rightSpeed) {
  m_drive->TankDrive(leftSpeed, rightSpeed);
}

units::meters_per_second_t DriveBase::GetLeftSpeed() {
  return units::meters_per_second_t(m_leftRearEncoder.GetVelocity());
}

units::meters_per_second_t DriveBase::GetRightSpeed() {
  return units::meters_per_second_t(m_rightRearEncoder.GetVelocity());
}

units::meter_t DriveBase::GetLeftDistance() {
  return units::meter_t(m_leftRearEncoder.GetPosition());
}

units::meter_t DriveBase::GetRightDistance() {
  return units::meter_t(m_rightRearEncoder.GetPosition());
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {
}
