// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveBase.h"

#include "Constants.h"

DriveBase::DriveBase()
    : leftFront(CANBusIds::SparkMax::Left_Front_No,
                rev::CANSparkMax::MotorType::kBrushless),
      leftRear(CANBusIds::SparkMax::Left_Rear_No,
               rev::CANSparkMax::MotorType::kBrushless),
      rightFront(CANBusIds::SparkMax::Right_Front_No,
                 rev::CANSparkMax::MotorType::kBrushless),
      rightRear(CANBusIds::SparkMax::Right_Rear_No,
                rev::CANSparkMax::MotorType::kBrushless) {
  SetSubsystem("Drivebase");
  leftMotors.SetInverted(true);
  SetCoastingEnabled(false);
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {
}

void DriveBase::SetCoastingEnabled(bool enabled) {
  rev::CANSparkMax::IdleMode mode =
      (enabled ? rev::CANSparkMax::IdleMode::kCoast
               : rev::CANSparkMax::IdleMode::kBrake);
  leftFront.SetIdleMode(mode);
  leftRear.SetIdleMode(mode);
  rightFront.SetIdleMode(mode);
  rightRear.SetIdleMode(mode);
}

void DriveBase::ArcadeDrive(double xaxisSpeed, double zaxisRotate,
                            double squareInputs) {
  drive.ArcadeDrive(xaxisSpeed, zaxisRotate, squareInputs);
}

void DriveBase::TankDrive(double leftSpeed, double rightSpeed) {
  drive.TankDrive(leftSpeed, rightSpeed);
}

void DriveBase::ResetEncoders() {
  leftFrontEncoder.SetPosition(0.0);
  leftRearEncoder.SetPosition(0.0);
  rightFrontEncoder.SetPosition(0.0);
  rightRearEncoder.SetPosition(0.0);
}

int DriveBase::GetLeftEncoderCount() {
  // TODO(mjh): Decide if we want to just use 1 encoder per side, or take an
  // average, or what.
  return leftFrontEncoder.GetPosition();
}

int DriveBase::GetRightEncoderCount() {
  // TODO(mjh): Decide if we want to just use 1 encoder per side, or take an
  // average, or what.
  return rightFrontEncoder.GetPosition();
}
