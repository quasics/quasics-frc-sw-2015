// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>
#include <units/length.h>
#include <units/time.h>

constexpr double kTicksPerRevolution_NeoMotor = 42;

constexpr double kGearRatio_2021 = 10.71;

static constexpr units::length::inch_t kWheelDiameter = 6.0_in;

Drivebase::Drivebase()
: m_odometry(units::degree_t(adiGyro.GetAngle())),
  leftFront(CANBusIds::SparkMaxIds::Left_Front_Number,
                rev::CANSparkMax::MotorType::kBrushless),
  leftRear(CANBusIds::SparkMaxIds::Left_Rear_Number,
               rev::CANSparkMax::MotorType::kBrushless),
  rightFront(CANBusIds::SparkMaxIds::Right_Front_Number,
                 rev::CANSparkMax::MotorType::kBrushless),
  rightRear(CANBusIds::SparkMaxIds::Right_Rear_Number,
                rev::CANSparkMax::MotorType::kBrushless) {
  SetSubsystem("Drivebase");

  rightFront.SetInverted(false);
  rightRear.SetInverted(false);

  LeftMotors.reset(new frc::SpeedControllerGroup(leftFront, leftRear));
  RightMotors.reset(new frc::SpeedControllerGroup(rightFront, rightRear));

  m_drive.reset(new frc::DifferentialDrive(*LeftMotors, *RightMotors));

  adiGyro.Calibrate();
}

// This method will be called once per scheduler run
void Drivebase::Periodic() {
    auto rotation = GetZAxisGyro().GetRotation2d();
  auto leftDistance = GetLeftEncoderDistance();
  auto rightDistance = GetRightEncoderDistance();

  frc::SmartDashboard::PutNumber("Right Side Encoders ",  leftDistance.to<double>());
  frc::SmartDashboard::PutNumber("Left Side Encoders",  rightDistance.to<double>());

  m_odometry.Update(rotation, leftDistance, rightDistance);
}

void Drivebase::setMotorSpeed(double leftSpeed, double rightSpeed){
    std::cerr << "Setting speeds: left=" << leftSpeed << ", right=" << rightSpeed << std::endl;
    leftFront.Set(leftSpeed * DrivebaseConstants::powerScaling);
    leftRear.Set(leftSpeed * DrivebaseConstants::powerScaling);
    rightFront.Set(rightSpeed * DrivebaseConstants::powerScaling);
    rightRear.Set(rightSpeed * DrivebaseConstants::powerScaling);
}

void Drivebase::Stop(){
    setMotorSpeed(0, 0);
}

void Drivebase::ResetEncoders() {
    leftFrontEncoder.SetPosition(0.0);
    leftRearEncoder.SetPosition(0.0);
    rightFrontEncoder.SetPosition(0.0);
    rightRearEncoder.SetPosition(0.0);
}

double Drivebase::GetLeftEncoderCount() {
    return leftFrontEncoder.GetPosition();
}

double Drivebase::GetRightEncoderCount() {
    return rightFrontEncoder.GetPosition();
}

frc::Pose2d Drivebase::GetPose(){
    return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds Drivebase::GetWheelSpeeds(){
    return frc::DifferentialDriveWheelSpeeds{leftRearEncoder.GetVelocity() * 1_m/1_s, rightRearEncoder.GetVelocity()* 1_m/1_s};
    
}

void Drivebase::ResetOdemetry(frc::Pose2d pose) {
    ResetEncoders();
    m_odometry.ResetPosition(pose, adiGyro.GetRotation2d());
}

void Drivebase::TankDriveVolts(units::volt_t left, units::volt_t right) {
    LeftMotors->SetVoltage(-left);
    RightMotors->SetVoltage(right);
    m_drive->Feed();
}
units::meter_t Drivebase::GetRightEncoderDistance(){
    auto distance = ((rightFrontEncoder.GetPosition()) / kGearRatio_2021) *
                       (kWheelDiameter * wpi::math::pi);
  return distance;
}
units::meter_t Drivebase::GetLeftEncoderDistance(){
    auto distance = ((leftFrontEncoder.GetPosition())/ kGearRatio_2021) *
                        (kWheelDiameter * wpi::math::pi);
    return distance;
}