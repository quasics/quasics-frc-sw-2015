// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/length.h>
#include <units/time.h>

#include <iostream>

#include "Constants.h"

#define LOG_EVERY_N_TIMES(n, outputCmd)       \
  {                                           \
    static int counter = -1;                  \
    if ((counter = (counter + 1) % n) == 0) { \
      outputCmd;                              \
    }                                         \
  }

constexpr double kTicksPerRevolution_NeoMotor = 42;

constexpr double kGearRatio_2021 = 10.71;

static constexpr units::length::inch_t kWheelDiameter = 6.0_in;

Drivebase::Drivebase()
    : m_odometry(units::degree_t(m_adiGyro.GetAngle())),
      m_leftFront(CANBusIds::SparkMaxIds::Left_Front_Number,
                        rev::CANSparkMax::MotorType::kBrushless),
      m_leftRear(CANBusIds::SparkMaxIds::Left_Rear_Number,
                       rev::CANSparkMax::MotorType::kBrushless),
      m_rightFront(CANBusIds::SparkMaxIds::Right_Front_Number,
                         rev::CANSparkMax::MotorType::kBrushless),
      m_rightRear(CANBusIds::SparkMaxIds::Right_Rear_Number,
                        rev::CANSparkMax::MotorType::kBrushless) {
  SetSubsystem("Drivebase");

  m_rightFront.SetInverted(true);
  m_rightRear.SetInverted(true);

  m_leftFront.SetInverted(false);
  m_leftRear.SetInverted(false);

  m_leftMotors.reset(new frc::SpeedControllerGroup(m_leftFront, m_leftRear));
  m_rightMotors.reset(new frc::SpeedControllerGroup(m_rightFront, m_rightRear));

  // Default for the encoders is to report velocity in RPM; we want that to come
  // back as m/s.
  units::meter_t wheelCircumference =
      kWheelDiameter * wpi::math::pi;  // Will auto-convert to meters
  units::meter_t velocityAdjustmentForGearing =
      wheelCircumference / kGearRatio_2021;  // Should convert RPM to m/min
  units::meter_t velocityAdjustment =
      velocityAdjustmentForGearing / 60;  // Adjust to m/s
  std::cout << "Wheel circumference: " << wheelCircumference << "\n"
            << "Velocity adjustment: " << velocityAdjustment << std::endl;

  m_leftFrontEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_leftRearEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_rightFrontEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());
  m_rightRearEncoder.SetVelocityConversionFactor(
      velocityAdjustment.to<double>());

  m_drive.reset(new frc::DifferentialDrive(*m_leftMotors, *m_rightMotors));

  ResetEncoders();
  m_adiGyro.Calibrate();
}

// This method will be called once per scheduler run
void Drivebase::Periodic() {
  auto rotation = GetZAxisGyro().GetRotation2d();
  auto leftDistance = GetLeftEncoderDistance();
  auto rightDistance = GetRightEncoderDistance();

  frc::SmartDashboard::PutNumber("L Encoders ", leftDistance.to<double>());
  frc::SmartDashboard::PutNumber("R Encoders", rightDistance.to<double>());
  frc::SmartDashboard::PutNumber("Rotation", rotation.Degrees().to<double>());

  m_odometry.Update(rotation, leftDistance, rightDistance);

  //   LOG_EVERY_N_TIMES(100, auto speeds = GetWheelSpeeds();
  //                     std::cout << "Speeds: " << speeds.left << " / "
  //                               << speeds.right << std::endl;)
}

void Drivebase::SetMotorSpeed(double leftSpeed, double rightSpeed) {
  //   LOG_EVERY_N_TIMES(50, std::cerr << "Setting speeds: left=" << leftSpeed
  //                                   << ", right=" << rightSpeed <<
  //                                   std::endl;)
  m_drive->TankDrive(leftSpeed, -rightSpeed);
}

void Drivebase::Stop() {
  SetMotorSpeed(0, 0);
}

void Drivebase::ResetEncoders() {
  m_leftFrontEncoder.SetPosition(0.0);
  m_leftRearEncoder.SetPosition(0.0);
  m_rightFrontEncoder.SetPosition(0.0);
  m_rightRearEncoder.SetPosition(0.0);
}

double Drivebase::GetLeftEncoderCount() {
  return m_leftFrontEncoder.GetPosition();
}

double Drivebase::GetRightEncoderCount() {
  return m_rightFrontEncoder.GetPosition();
}

frc::Pose2d Drivebase::GetPose() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds Drivebase::GetWheelSpeeds() {
  return frc::DifferentialDriveWheelSpeeds{
      m_leftRearEncoder.GetVelocity() * 1_m / 1_s,
      m_rightRearEncoder.GetVelocity() * 1_m / 1_s};
}

void Drivebase::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, m_adiGyro.GetRotation2d());
}

void Drivebase::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors->SetVoltage(left);
  m_rightMotors->SetVoltage(right);
  m_drive->Feed();
}
units::meter_t Drivebase::GetRightEncoderDistance() {
  auto distance = ((m_rightFrontEncoder.GetPosition()) / kGearRatio_2021) *
                  (kWheelDiameter * wpi::math::pi);
  return distance;
}
units::meter_t Drivebase::GetLeftEncoderDistance() {
  auto distance = ((m_leftFrontEncoder.GetPosition()) / kGearRatio_2021) *
                  (kWheelDiameter * wpi::math::pi);
  return distance;
}