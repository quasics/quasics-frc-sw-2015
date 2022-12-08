// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveBase.h"

#include <ctre/phoenix/sensors/WPI_Pigeon2.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>
// #include <wpi/numbers> // TODO(mjh): Report breakage
#include <numbers>

// Conditional compilation flags, controlling what sort of a gyro we shoud try
// to set up for the drive base.  Only one should be enabled at a time; if more
// than 1 is enabled, then only 1 will take effect.
#define ENABLE_PIGEON2_GYRO
// #define ENABLE_ADXRS450_GYRO

DriveBase::DriveBase() {
  SetName("DriveBase");

  /////////////////////////
  // Motor and encoder setup

  // Configure the motors.  (Must be done before adding them to motor
  // controller groups.)
  m_leftFront.SetInverted(true);
  m_leftRear.SetInverted(true);
  m_rightFront.SetInverted(false);
  m_rightRear.SetInverted(false);

  // Build the motor controller groups and differential drive.
  m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftRear));
  m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightRear));

  m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));

  // Set up the encoders so that they report units that we care about.
  ConfigureEncoders();

  /////////////////////////
  // Gyro setup

  // Allocate a gyro (if one is enabled)
#if defined(ENABLE_PIGEON2_GYRO)
  m_gyro.reset(new ctre::phoenix::sensors::WPI_Pigeon2{PIGEON2_CAN_ID});
#elif defined(ENABLE_ADXRS450_GYRO)
  // Assume that the Chip Select jumper is set to CS0
  m_gyro.reset(new frc::ADXRS450_Gyro{frc::SPI::Port::kOnboardCS0});
#else
  std::cerr
      << "**** WARNING: No gyro is enabled for use with the drive base!\n";
#endif  // ENABLE_ADXRS450_GYRO

  // If we allocated a gyro, calibrate & reset it.
  if (m_gyro) {
    m_gyro->Calibrate();
    m_gyro->Reset();
  }
}

/**
 * Configures the encoders so that they will report back speeds and positions in
 * units that we generally care about (e.g., m/s and meters, rather than RPM and
 * fractions of a rotation).
 *
 * TODO: Check the math on all of this (at least empirically).
 */
void DriveBase::ConfigureEncoders() {
  // Compute the distance (in meters) for one rotation of the wheel, first.
  const units::meter_t wheelCircumference =
      WHEEL_DIAMETER_INCHES *  // Will auto-convert to meters! :-)
      std::numbers::pi;
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

/**
 * Resets the encoders to 0.  (Similar to resetting the trip odometer in a car.)
 *
 * Warning: this can mess with odometry (once it's implemented), so please be
 * very careful about invoking this, or else reset the odometry handling as
 * well.
 */
void DriveBase::ResetEncoders() {
  m_leftFrontEncoder.SetPosition(0);
  m_rightFrontEncoder.SetPosition(0);
  m_leftRearEncoder.SetPosition(0);
  m_rightRearEncoder.SetPosition(0);
}

/**
 * Enables/disables "coast mode".
 *
 * In "coast" mode", the motors will allow the wheels to roll freely when the
 * power is 0; when it is disabled, they will effectively put on the brakes when
 * power is 0.
 */
void DriveBase::SetCoastingEnabled(bool tf) {
  const rev::CANSparkMax::IdleMode mode =
      tf ? rev::CANSparkMax::IdleMode::kCoast
         : rev::CANSparkMax::IdleMode::kBrake;
  m_leftFront.SetIdleMode(mode);
  m_rightFront.SetIdleMode(mode);
  m_leftRear.SetIdleMode(mode);
  m_rightRear.SetIdleMode(mode);
}

/**
 * Sets power via "tank mode", with the left and right sides controlled
 * independently.
 *
 * @param leftSpeed  % power to be applied to left side (-1.0 to +1.0)
 * @param rightSpeed % power to be applied to right side (-1.0 to +1.0)
 */
void DriveBase::TankDrive(double leftSpeed, double rightSpeed) {
  m_drive->TankDrive(leftSpeed, rightSpeed);
}

/** @return the current speed of the left side motors, in meters/sec. */
units::meters_per_second_t DriveBase::GetLeftSpeed() {
  return units::meters_per_second_t(m_leftRearEncoder.GetVelocity());
}

/** @return the current speed of the right side motors, in meters/sec. */
units::meters_per_second_t DriveBase::GetRightSpeed() {
  return units::meters_per_second_t(m_rightRearEncoder.GetVelocity());
}

/**
 * @return the current distance traveled by the left side wheels, in meters.
 *
 * @see #ResetEncoders()
 */
units::meter_t DriveBase::GetLeftDistance() {
  return units::meter_t(m_leftRearEncoder.GetPosition());
}

/**
 * @return the current distance traveled by the right side wheels, in meters.
 *
 * @see #ResetEncoders()
 */
units::meter_t DriveBase::GetRightDistance() {
  return units::meter_t(m_rightRearEncoder.GetPosition());
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {
  // Log the heading (for debugging purposes).
  frc::SmartDashboard::PutNumber("Heading (deg)", GetHeading());
}
