// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IDrivebase.h"

#include <frc/smartdashboard/SmartDashboard.h>

IDrivebase::IDrivebase(){};

// This method will be called once per scheduler run
void IDrivebase::Periodic() {
  updateOdometry();

  frc::Pose2d pose = getPose();
  frc::SmartDashboard::PutNumber("x", double(pose.X()));

  frc::SmartDashboard::PutNumber("y", double(pose.Y()));

  frc::SmartDashboard::PutNumber("angle", double(pose.Rotation().Degrees()));

  frc::SmartDashboard::PutNumber("left Encoder meters",
                                 double(getLeftEncoder().getPosition()));

  frc::SmartDashboard::PutNumber("right Encoder meters",
                                 double(getRightEncoder().getPosition()));
}

void IDrivebase::tankDrive(double leftInputPercent, double rightInputPercent) {
  // Make sure that speeds are limited to a range of -1 to +1
  double leftPercent = std::max(std::min(1.0, leftInputPercent), -1.0);
  double rightPercent = std::max(std::min(1.0, rightInputPercent), -1.0);

  // Other adjustments could happen here (e.g., turbo/turtle mode, or deadband,
  // etc.)

  // Apply the speeds to the motors!
  setMotorSpeeds(leftPercent, rightPercent);
}

frc2::CommandPtr IDrivebase::sysIdQuasistatic(
    frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr IDrivebase::sysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}