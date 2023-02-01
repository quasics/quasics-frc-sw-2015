// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveBase.h"

#include <cmath>

DriveBase::DriveBase() {
  SetName("DriveBase");

  m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftRear));
  m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightRear));

  m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {}

void DriveBase::TankDrive(double leftPercent, double rightPercent) {
  double scaledLeft = MAX_DRIVE_SPEED_SCALING_FACTOR * std::max(-1.0, std::min(+1.0, leftPercent));
  double scaledRight = MAX_DRIVE_SPEED_SCALING_FACTOR * std::max(-1.0, std::min(+1.0, rightPercent));
  m_drive->TankDrive(scaledLeft, scaledRight);
}
