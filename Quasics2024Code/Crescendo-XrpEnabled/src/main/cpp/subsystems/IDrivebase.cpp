// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IDrivebase.h"

#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

DeadBandEnforcer IDrivebase::m_voltageDeadbandEnforcer(-0.001);

IDrivebase::IDrivebase()
    :  // TODO: (Rylie) Please confirm that this is the actual track width for
       // Margaret. (And you should also figure out how we can make sure that
       // it's not going to be a problem if we switch to a drive base with a
       // different size (like Mae or Sally) for testing.)
      m_kinematics{TRACK_WIDTH_METERS_MARGARET} {
}

// This method will be called once per scheduler run
void IDrivebase::Periodic() {
  updateOdometry();
#ifndef ENABLE_COMPETITION_ROBOT
  frc::Pose2d pose = getPose();
  frc::SmartDashboard::PutNumber("x", double(pose.X()));

  frc::SmartDashboard::PutNumber("y", double(pose.Y()));

  frc::SmartDashboard::PutNumber("angle", double(pose.Rotation().Degrees()));
  frc::SmartDashboard::PutNumber("raw angle", getYaw().value());

  frc::SmartDashboard::PutNumber("left Encoder meters",
                                 double(getLeftEncoder_HAL().getPosition()));

  frc::SmartDashboard::PutNumber("right Encoder meters",
                                 double(getRightEncoder_HAL().getPosition()));
#endif
}

void IDrivebase::tankDrive(double leftInputPercent, double rightInputPercent) {
  // Make sure that speeds are limited to a range of -1 to +1
  double leftPercent = std::max(std::min(1.0, leftInputPercent), -1.0);
  double rightPercent = std::max(std::min(1.0, rightInputPercent), -1.0);

  // Other adjustments could happen here (e.g., turbo/turtle mode, or deadband,
  // etc.)

  // Apply the speeds to the motors!
  setMotorSpeeds_HAL(leftPercent, rightPercent);
}

frc2::CommandPtr IDrivebase::sysIdQuasistatic(
    frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr IDrivebase::sysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}

void IDrivebase::arcadeDrive(units::meters_per_second_t fSpeed,
                             units::radians_per_second_t rSpeed) {
  frc::ChassisSpeeds speeds;
  speeds.vx = fSpeed;
  speeds.omega = rSpeed;
  const auto wheelSpeeds = m_kinematics.ToWheelSpeeds(speeds);

  setMotorSpeeds_HAL(wheelSpeeds.left / RobotConstants::MAX_SPEED,
                     wheelSpeeds.right / RobotConstants::MAX_SPEED);
}

double IDrivebase::convertVoltageToPercentSpeed(units::volt_t volts) {
  const double voltageInput = frc::RobotController::GetInputVoltage();
  const double metersPerSec = (volts.value() / voltageInput);
  const double speedPercentage = m_voltageDeadbandEnforcer(
      metersPerSec / RobotConstants::MAX_SPEED.value());
  return speedPercentage;
}

units::volt_t IDrivebase::convertPercentSpeedToVoltage(double percentSpeed) {
  const double referenceVoltage = frc::RobotController::GetInputVoltage();
  return (referenceVoltage * percentSpeed) * 1_V;
}
