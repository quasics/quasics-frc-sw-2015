/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveBase.h"
#include "commands/TankDrive.h"

constexpr int Left_Front_No = 4;
constexpr int Left_Rear_No = 3;
constexpr int Right_Front_No = 2;
constexpr int Right_Rear_No = 1;

DriveBase::DriveBase() : Subsystem("DriveBase"),
  leftFront(Left_Front_No, rev::CANSparkMax::MotorType::kBrushless),
  leftRear(Left_Rear_No, rev::CANSparkMax::MotorType::kBrushless),
  rightFront(Right_Front_No, rev::CANSparkMax::MotorType::kBrushless),
  rightRear(Right_Rear_No, rev::CANSparkMax::MotorType::kBrushless){
    leftFront.SetInverted(true);
    leftRear.SetInverted(true);
  }

void DriveBase::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  SetDefaultCommand(new TankDrive());
}

void DriveBase::SetMotorSpeed(double LeftSpeed, double RightSpeed){
  leftFront.Set(LeftSpeed);
  leftRear.Set(LeftSpeed);
  rightFront.Set(RightSpeed);
  rightRear.Set(RightSpeed);
}

void DriveBase::Stop(){
  leftFront.Set(0);
  leftRear.Set(0);
  rightFront.Set(0);
  rightRear.Set(0);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
