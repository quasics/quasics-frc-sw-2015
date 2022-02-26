// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"

#include <rev/CANSparkMax.h>

#include <iostream>
#include <wpi/numbers>

// Note: Mae appears to have the left motors marked as "Inverted" at the
// firmware settings/configuration level, and thus we need to override
// that when we're trying to talk to it in the same way that we do
// Nike and Sally.
#undef TARGETING_MAE_2021_ROBOT

Drivebase::Drivebase() {
  SetName("Drivebase");

  m_rightFront.SetInverted(true);
  m_rightBack.SetInverted(true);

#ifdef TARGETING_MAE_2021_ROBOT
  m_leftBack.SetInverted(false);
  m_leftFront.SetInverted(false);
#endif

  m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftBack));
  m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightBack));

  m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));

  m_gyro.Calibrate();
  m_gyro.Reset();

  ConfigureEncoders();
}

// This method will be called once per scheduler run

void Drivebase::ConfigureEncoders() {
  // Calculate wheel circumference (distance travelled per wheel revolution).
  const units::meter_t wheelCircumference = WHEEL_DIAMETER * wpi::numbers::pi;

  // Compute distance traveled per rotation of the motor.
  const units::meter_t gearingConversion =
      wheelCircumference / DRIVEBASE_GEAR_RATIO;

  // Compute conversion factor (used to change "(motor) RPM" to "m/sec").
  const units::meter_t velocityCorrection = gearingConversion / 60;

  // Update encoders so that they will report distance as meters traveled,
  // rather than rotations.
  m_leftFrontEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_leftBackEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_rightFrontEncoder.SetPositionConversionFactor(gearingConversion.value());
  m_rightBackEncoder.SetPositionConversionFactor(gearingConversion.value());

  // Update encoders so that they will report velocity as m/sec, rather than
  // RPM.
  m_leftFrontEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_leftBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_rightFrontEncoder.SetVelocityConversionFactor(velocityCorrection.value());
  m_rightBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());

  ResetEncoders();
}

void Drivebase::ResetEncoders() {
  m_leftFrontEncoder.SetPosition(0);
  m_rightFrontEncoder.SetPosition(0);
  m_leftBackEncoder.SetPosition(0);
  m_rightBackEncoder.SetPosition(0);
}

void Drivebase::Periodic() {
  auto rotation = m_gyro.GetRotation2d();
  auto leftDistance = GetLeftDistance();
  auto rightDistance = GetRightDistance();
  // std::cout << "rotation = " << rotation.Degrees().value()
  //           << "degrees, leftDistance = " << leftDistance.value()
  //           << "m, rightDistance = " << rightDistance.value() << "m"
  //           << std::endl;

  m_odometry.Update(rotation, leftDistance, rightDistance);
}

void Drivebase::SetMotorPower(double leftPower, double rightPower) {
  m_drive->TankDrive(leftPower, rightPower);
}

units::meter_t Drivebase::GetLeftDistance() {
  // Note that the conversion factor configured earlier means that we're getting
  // position in meters.
  return units::meter_t(m_leftFrontEncoder.GetPosition());
}

units::meter_t Drivebase::GetRightDistance() {
  // Note that the conversion factor configured earlier means that we're getting
  // position in meters.
  return units::meter_t(m_rightFrontEncoder.GetPosition());
}

units::meters_per_second_t Drivebase::GetLeftVelocity() {
  // Note that the conversion factor configured earlier means that we're getting
  // velocity in m/sec.
  return units::meters_per_second_t(m_leftFrontEncoder.GetVelocity());
}

units::meters_per_second_t Drivebase::GetRightVelocity() {
  // Note that the conversion factor configured earlier means that we're getting
  // velocity in m/sec.
  return units::meters_per_second_t(m_rightFrontEncoder.GetVelocity());
}
frc::Pose2d Drivebase::GetPose() {
  return m_odometry.GetPose();
}
void Drivebase::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, m_gyro.GetRotation2d());
}

void Drivebase::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftSide->SetVoltage(left);
  m_rightSide->SetVoltage(right);
  m_drive->Feed();
}

frc::DifferentialDriveWheelSpeeds Drivebase::GetWheelSpeeds() {
  return frc::DifferentialDriveWheelSpeeds{GetLeftVelocity(),
                                           GetRightVelocity()};
}