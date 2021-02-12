// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveForward.h"

#include <units/math.h>

#include <iostream>

// Called when the command is initially scheduled.
void DriveForward::Initialize() {
  if (m_useGyro) {
    m_driveTrain->ResetGyro();
  } else {
    m_driveTrain->ResetEncoders();
  }
  m_integral = 0;
  m_lastError = 0;
}

double DriveForward::GetSteeringError() {
  double error;
  if (m_useGyro) {
    const double rawAngle = m_driveTrain->GetGyroAngleZ();
    const double clampedAngle = double(int(rawAngle) % 360);
    const double adjustedAngle =
        (clampedAngle <= 180) ? clampedAngle : clampedAngle - 360;
    error = -(adjustedAngle / 180.0);  // our target delta for the angle is zero
    if (m_noisy) {
      std::cout << "rawAngle: " << rawAngle << ", clamped: " << clampedAngle
                << ", adjusted: " << adjustedAngle;
    }
  } else {
    const auto distanceLeft = units::math::abs(m_driveTrain->GetLeftDistance());
    const auto distanceRight =
        units::math::abs(m_driveTrain->GetRightDistance());
    error =
        -double(distanceLeft - distanceRight);  // our target delta for distance
                                                // between the two sides is zero
    if (m_noisy) {
      std::cout << "distanceLeft: " << distanceLeft
                << ", distanceRight: " << distanceRight;
    }
  }
  return error;
}

// Called repeatedly when this Command is scheduled to run
void DriveForward::Execute() {
  const double error = GetSteeringError();
  m_integral += (error * kCurveInterval);
  double derivative = ((error - m_lastError) / kCurveInterval);

  const double turnPower = (kP * error)          // Proportional
                           + (kI * m_integral)   // Integral
                           + (kD * derivative);  // Derivative

  if (m_noisy) {
    std::cout << ", error: " << error << ", integral: " << m_integral
              << ", derivative: " << derivative << ", turnPower: " << turnPower
              << std::endl;
  }

  m_driveTrain->ArcadeDrive(m_power, turnPower, false);
}

// Called once the command ends or is interrupted.
void DriveForward::End(bool interrupted) {
  m_driveTrain->Stop();
}
