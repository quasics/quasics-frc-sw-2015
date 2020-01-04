/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveBase.h"

constexpr int Left_Front_No = 4;
constexpr int Left_Rear_No = 3;
constexpr int Right_Front_No = 2;
constexpr int Right_Rear_No = 1;

DriveBase::DriveBase()
    : leftFront(Left_Front_No, rev::CANSparkMax::MotorType::kBrushless),
      leftRear(Left_Rear_No, rev::CANSparkMax::MotorType::kBrushless),
      rightFront(Right_Front_No, rev::CANSparkMax::MotorType::kBrushless),
      rightRear(Right_Rear_No, rev::CANSparkMax::MotorType::kBrushless) {
  SetSubsystem("DriveBase");
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {}

void DriveBase::SetMotorPower(double leftPower, double rightPower) {
  leftFront.Set(leftPower);
  leftRear.Set(leftPower);
  rightFront.Set(rightPower);
  rightRear.Set(rightPower);
}