// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveForward.h"

#include <iostream>
#include <units/math.h>

const double kP = 0.02;
constexpr bool useGyroForAngles = false;

// Called when the command is initially scheduled.
void DriveForward::Initialize()
{
  if (useGyroForAngles)
  {
    m_driveTrain->ResetGyro();
  }
  else
  {
    m_driveTrain->ResetEncoders();
  }
}

// Called repeatedly when this Command is scheduled to run
void DriveForward::Execute()
{
  double turnPower;
  if (useGyroForAngles)
  {
    const double rawAngle = m_driveTrain->GetGyroAngleZ();
    const double clampedAngle = double(int(rawAngle) % 360);
    const double adjustedAngle = (clampedAngle <= 180)
                                     ? clampedAngle
                                     : clampedAngle - 360;
    const double error = -adjustedAngle; // our target delta is zero
    std::cout << "rawAngle: " << rawAngle
              << ", clamped: " << clampedAngle
              << ", adjusted: " << adjustedAngle
              << ", error: " << error;
    turnPower = kP * error;
  }
  else
  {
    const auto distanceLeft = units::math::abs(m_driveTrain->GetLeftDistance());
    const auto distanceRight = units::math::abs(m_driveTrain->GetRightDistance());
    const double error = -double(distanceLeft - distanceRight);
    std::cout << "distanceLeft: " << distanceLeft
              << ", distanceRight: " << distanceRight
              << ", error: " << error;
    turnPower = error;
  }
  std::cout << ", turnPower: " << turnPower
            << std::endl;

  m_driveTrain->ArcadeDrive(m_power, turnPower, false);
  // const double error = -gyroAngle #Our target angle is zero
  //                          turn_power = kP * error
  //                                                drive.arcadeDrive(power, turn_power, squaredInputs = False)
}

// Called once the command ends or is interrupted.
void DriveForward::End(bool interrupted)
{
  m_driveTrain->Stop();
}
