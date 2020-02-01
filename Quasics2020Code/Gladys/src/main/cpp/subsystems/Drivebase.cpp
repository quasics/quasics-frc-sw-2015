/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivebase.h"
#include "Constants.h"

Drivebase::Drivebase()
: leftFront(DriveBaseConstants::Left_Front_No, rev::CANSparkMax::MotorType::kBrushless),
  rightFront(DriveBaseConstants::Right_Front_No, rev::CANSparkMax::MotorType::kBrushless), 
  leftRear(DriveBaseConstants::Left_Rear_No, rev::CANSparkMax::MotorType::kBrushless),
  rightRear(DriveBaseConstants::Right_Rear_No, rev::CANSparkMax::MotorType::kBrushless){}

// This method will be called once per scheduler run
void Drivebase::Periodic() {}

void Drivebase::Stop(){
  SetPower(0, 0);
}

void Drivebase::SetPower(double rightPower, double leftPower){
    rightFront.Set(rightPower * DriveBaseConstants::MaxPower);
    rightRear.Set(rightPower * DriveBaseConstants::MaxPower);
    leftFront.Set(leftPower * DriveBaseConstants::MaxPower);
    leftRear.Set(leftPower * DriveBaseConstants::MaxPower);
}