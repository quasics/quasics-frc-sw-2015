// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"
#include "Constants.h"

Drivebase::Drivebase()
: leftFront(CANBusIds::SparkMaxIds::Left_Front_Number,
                rev::CANSparkMax::MotorType::kBrushless),
  leftRear(CANBusIds::SparkMaxIds::Left_Rear_Number,
               rev::CANSparkMax::MotorType::kBrushless),
  rightFront(CANBusIds::SparkMaxIds::Right_Front_Number,
                 rev::CANSparkMax::MotorType::kBrushless),
  rightRear(CANBusIds::SparkMaxIds::Right_Rear_Number,
                rev::CANSparkMax::MotorType::kBrushless) {
  SetSubsystem("Drivebase");
};

// This method will be called once per scheduler run
void Drivebase::Periodic() {}

void Drivebase::setMotorSpeed(double leftSpeed, double rightSpeed){
    leftFront.Set(leftSpeed);
    leftRear.Set(leftSpeed);
    rightFront.Set(-rightSpeed);
    rightRear.Set(-rightSpeed);
}

void Drivebase::Stop(){
    setMotorSpeed(0, 0);
}