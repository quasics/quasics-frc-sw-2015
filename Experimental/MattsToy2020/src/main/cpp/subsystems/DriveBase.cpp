/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveBase.h"

#include "Constants.h"

DriveBase::DriveBase()
    : leftFront(CANBusConstants::SparkMaxIds::DriveBaseLeftFrontId,
                rev::CANSparkMax::MotorType::kBrushless),
      leftRear(CANBusConstants::SparkMaxIds::DriveBaseLeftRearId,
               rev::CANSparkMax::MotorType::kBrushless),
      rightFront(CANBusConstants::SparkMaxIds::DriveBaseRightFrontId,
                 rev::CANSparkMax::MotorType::kBrushless),
      rightRear(CANBusConstants::SparkMaxIds::DriveBaseRightRearId,
                rev::CANSparkMax::MotorType::kBrushless) {
  SetSubsystem("DriveBase");
}

// This method will be called once per scheduler run
void DriveBase::Periodic() {}

void DriveBase::SetMotorPower(double leftPower, double rightPower) {
  leftFront.Set(leftPower * powerScaling);
  leftRear.Set(leftPower * powerScaling);
  rightFront.Set(rightPower * powerScaling);
  rightRear.Set(rightPower * powerScaling);
}