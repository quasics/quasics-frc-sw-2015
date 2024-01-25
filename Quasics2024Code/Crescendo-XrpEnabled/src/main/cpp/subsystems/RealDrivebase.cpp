// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RealDrivebase.h"

RealDrivebase::RealDrivebase() {
  SetName("RealDrivebase");
  // This is where we'd do any necessary motor configuration (e.g., setting some
  // as "inverted", etc.).
  configureEncoders();
  ResetOdometry(frc::Pose2d());
}

void RealDrivebase::setMotorSpeeds(double leftPercent, double rightPercent) {
  m_leftBack.Set(leftPercent);
  m_rightBack.Set(rightPercent);
}

void RealDrivebase::configureEncoders() {
  double pi = 3.141592;
  // Calculate wheel circumference (distance travelled per wheel revolution).
  const units::meter_t wheelCircumference = RobotPhysics::WHEEL_DIAMETER * pi;

  // Compute distance traveled per rotation of the motor.
  const units::meter_t gearingConversion =
      wheelCircumference / RobotPhysics::DRIVEBASE_GEAR_RATIO;

  // Compute conversion factor (used to change "(motor) RPM" to "m/sec").
  const units::meter_t velocityCorrection = gearingConversion / 60;

  // Update encoders so that they will report distance as meters traveled,
  // rather than rotations.
  m_leftBackEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_rightBackEncoder.SetPositionConversionFactor(gearingConversion.value());

  // Update encoders so that they will report velocity as m/sec, rather than
  // RPM.
  m_leftBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_rightBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());

  resetEncoders();
}

void RealDrivebase::tankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftBack.SetVoltage(left);
  m_rightBack.SetVoltage(right);
}