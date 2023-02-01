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
  std::unique_ptr<frc::MotorControllerGroup> m_leftSide(new frc::MotorControllerGroup(m_leftFront, m_leftRear));
  std::unique_ptr<frc::MotorControllerGroup> m_rightSide(new frc::MotorControllerGroup(m_rightFront, m_rightRear));
  m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));
}

units::degree_t DriveBase::GetAngle()
{
  // TODO(mjh): Implement this!
  return units::degree_t(0);
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {}

void DriveBase::TankDrive(double leftPercent, double rightPercent)
{
  double scaledLeft = MAX_DRIVE_SPEED_SCALING_FACTOR * std::max(-1.0, std::min(+1.0, leftPercent));
  double scaledRight = MAX_DRIVE_SPEED_SCALING_FACTOR * std::max(-1.0, std::min(+1.0, rightPercent));
  m_drive->TankDrive(scaledLeft, scaledRight);
}
