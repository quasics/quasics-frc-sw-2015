// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RealDrivebase.h"

#include <numbers>

#include "Constants.h"

RealDrivebase::RealDrivebase()
    : m_leftBack{MotorIds::SparkMax::LEFT_BACK_DRIVE_MOTOR_ID,
                 rev::CANSparkMax::MotorType::kBrushless},
      m_leftBackFollower{MotorIds::SparkMax::LEFT_FRONT_DRIVE_MOTOR_ID,
                         rev::CANSparkMax::MotorType::kBrushless},
      m_rightBackFollower{MotorIds::SparkMax::RIGHT_FRONT_DRIVE_MOTOR_ID,
                          rev::CANSparkMax::MotorType::kBrushless},
      m_rightBack{MotorIds::SparkMax::RIGHT_BACK_DRIVE_MOTOR_ID,
                  rev::CANSparkMax::MotorType::kBrushless},
      m_realGyro{SensorIds::PIGEON_CAN_ID} {
  SetName("RealDrivebase");
  // This is where we'd do any necessary motor configuration (e.g., setting some
  // as "inverted", etc.).
  configureEncoders();
  resetOdometry(frc::Pose2d());
}

void RealDrivebase::setMotorSpeeds_HAL(double leftPercent,
                                       double rightPercent) {
  m_leftBack.Set(leftPercent);
  m_leftBackFollower.Set(leftPercent);
  m_rightBack.Set(rightPercent);
  m_rightBackFollower.Set(rightPercent);
}

void RealDrivebase::configureEncoders() {
  // Calculate wheel circumference (distance travelled per wheel revolution).
  const units::meter_t wheelCircumference =
      RobotPhysics::WHEEL_DIAMETER * std::numbers::pi;

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

void RealDrivebase::setMotorVoltages_HAL(units::volt_t leftPower,
                                         units::volt_t rightPower) {
  m_leftBack.SetVoltage(leftPower);
  m_leftBackFollower.SetVoltage(leftPower);
  m_rightBack.SetVoltage(rightPower);
  m_rightBackFollower.SetVoltage(rightPower);
}