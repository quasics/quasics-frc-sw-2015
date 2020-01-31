/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivebase.h"


Drivebase::Drivebase()
: leftFront(1, rev::CANSparkMax::MotorType::kBrushless),
  rightFront(2, rev::CANSparkMax::MotorType::kBrushless), 
  leftRear(3, rev::CANSparkMax::MotorType::kBrushless),
  rightRear(4, rev::CANSparkMax::MotorType::kBrushless){}

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