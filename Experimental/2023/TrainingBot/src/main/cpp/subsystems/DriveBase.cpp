// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveBase.h"

#include <cmath>
#include <numbers>
#include <units/time.h>
#include <units/velocity.h>

DriveBase::DriveBase()
{
  SetName("DriveBase");

  // Things that may change from year to year.
  const double GEAR_RATIO = RobotConstants::DRIVE_BASE_GEAR_RATIO_2023; // Also used for Sally

  // Update velocity reporting to be in meters (vs. rotations)
  const units::meter_t wheelCircumference = std::numbers::pi * RobotConstants::WHEEL_DIAMETER;
  const units::meter_t distancePerRotation = wheelCircumference / GEAR_RATIO;
  m_leftFrontEncoder.SetPositionConversionFactor(distancePerRotation.value());
  m_rightFrontEncoder.SetPositionConversionFactor(distancePerRotation.value());

  // Update velocity reporting from RPM to m/s
  const units::meters_per_second_t velocityConversionFactor = distancePerRotation // converting revolutions to meters
                                                              / 60_s;             // converting minutes to seconds
  m_leftFrontEncoder.SetVelocityConversionFactor(velocityConversionFactor.value());
  m_rightFrontEncoder.SetVelocityConversionFactor(velocityConversionFactor.value());

  // Build the differential drive
  m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftRear));
  m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightRear));
  m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));

  // Set up the gyro
  m_gyro.reset(new ctre::phoenix::sensors::WPI_Pigeon2{SensorIds::PIGEON2_CAN_ID});
  m_gyro->Calibrate();
  m_gyro->Reset();

  // This *shouldn't* be needed, but I'm leaving us more gap between when the motors
  // are "fed" before the MotorSafety watchdog declares a failure.
  m_drive->SetExpiration(1_s);
  // m_drive->SetSafetyEnabled(false);  // Uncomment to disable motor safety checks completely.
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {}

void DriveBase::TankDrive(double leftPercent, double rightPercent)
{
  double scaledLeft = MAX_DRIVE_SPEED_SCALING_FACTOR * std::max(-1.0, std::min(+1.0, leftPercent));
  double scaledRight = MAX_DRIVE_SPEED_SCALING_FACTOR * std::max(-1.0, std::min(+1.0, rightPercent));
  m_drive->TankDrive(scaledLeft, scaledRight);
}
