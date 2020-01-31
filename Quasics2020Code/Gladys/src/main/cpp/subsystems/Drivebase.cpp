/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivebase.h"

// CAN bus IDs for the 4 motors used for the drive base.
constexpr int Left_Front_No = 4;
constexpr int Left_Rear_No = 3;
constexpr int Right_Front_No = 2;
constexpr int Right_Rear_No = 1;

Drivebase::Drivebase()
: leftFront(Left_Front_No, rev::CANSparkMax::MotorType::kBrushless),
  rightFront(Right_Front_No, rev::CANSparkMax::MotorType::kBrushless), 
  leftRear(Left_Rear_No, rev::CANSparkMax::MotorType::kBrushless),
  rightRear(Right_Rear_No, rev::CANSparkMax::MotorType::kBrushless){}

// This method will be called once per scheduler run
void Drivebase::Periodic() {}
  void Drivebase::Stop(){SetPower(0, 0);}
  void Drivebase::SetPower(double rightPower, double leftPower){
    const double maxPower = .45;
      rightFront.Set(rightPower * maxPower);
      rightRear.Set(rightPower * maxPower);
      leftFront.Set(leftPower * maxPower);
      leftRear.Set(leftPower * maxPower);
  }