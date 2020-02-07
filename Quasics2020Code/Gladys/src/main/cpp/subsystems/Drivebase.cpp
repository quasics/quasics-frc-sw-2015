/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivebase.h"

Drivebase::Drivebase() 
    : leftFront(CANBusIds::SparkMax::Left_Front_No, rev::CANSparkMax::MotorType::kBrushless),
      leftRear(CANBusIds::SparkMax::Left_Rear_No, rev::CANSparkMax::MotorType::kBrushless),
      rightFront(CANBusIds::SparkMax::Right_Front_No, rev::CANSparkMax::MotorType::kBrushless),
      rightRear(CANBusIds::SparkMax::Right_Rear_No, rev::CANSparkMax::MotorType::kBrushless){
    SetSubsystem("Drivebase");
}
// This method will be called once per scheduler run
void Drivebase::Periodic() {}

void Drivebase::SetMotorPower(double leftPower, double rightPower) {
    leftFront.Set(leftPower * powerScaling);
    leftRear.Set(leftPower * powerScaling);
    rightFront.Set(rightPower * powerScaling);
    rightRear.Set(rightPower * powerScaling);
}