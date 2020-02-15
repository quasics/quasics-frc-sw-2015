/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <iostream>
#include "subsystems/Drivebase.h"

Drivebase::Drivebase() 
    : leftFront(CANBusIds::SparkMax::Left_Front_No, rev::CANSparkMax::MotorType::kBrushless),
      leftRear(CANBusIds::SparkMax::Left_Rear_No, rev::CANSparkMax::MotorType::kBrushless),
      rightFront(CANBusIds::SparkMax::Right_Front_No, rev::CANSparkMax::MotorType::kBrushless),
      rightRear(CANBusIds::SparkMax::Right_Rear_No, rev::CANSparkMax::MotorType::kBrushless){
    SetSubsystem("Drivebase");
    leftFront.SetInverted(true);
    leftRear.SetInverted(true);
}
// This method will be called once per scheduler run
void Drivebase::Periodic() {}


  void Drivebase::DisplayEncoderValues() {
    frc::SmartDashboard::PutNumber("Left Front Encoder Position", leftFrontEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Left Front Encoder Velocity", leftFrontEncoder.GetVelocity());

    frc::SmartDashboard::PutNumber("Left Rear Encoder Position", leftRearEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Left Rear Encoder Velocity", leftRearEncoder.GetVelocity());

    frc::SmartDashboard::PutNumber("Right Front Encoder Position", rightFrontEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Right Front Encoder Velocity", rightFrontEncoder.GetVelocity());

    frc::SmartDashboard::PutNumber("Right Rear Encoder Position", rightRearEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Right Rear Encoder Velocity", rightRearEncoder.GetVelocity());

    //There are 42 encoder ticks in a revolution. (The output from .GetPosition() is in ticks)
    // formula for encoder value to inches: (encoder output in ticks)/(42 ticks)/(10.71 revolutions)*(6Pi inches forward)
  }

  double Drivebase::GetLeftFrontEncoderPosition() {
    // Note: we're negating the values to match live behavior (e.g., so
    // that we get positive numbers when the motor is moving us forward).
    return -leftFrontEncoder.GetPosition();
  }

  double Drivebase::GetRightFrontEncoderPosition() {
    // Note: we're negating the values to match live behavior (e.g., so
    // that we get positive numbers when the motor is moving us forward).
    return -rightFrontEncoder.GetPosition();
  }

  void Drivebase::ResetEncoderPositions() {
    leftFrontEncoder.SetPosition(0.0);
    leftRearEncoder.SetPosition(0.0);
    rightFrontEncoder.SetPosition(0.0);
    rightRearEncoder.SetPosition(0.0);
  }


void Drivebase::SetMotorPower(double rightPower, double leftPower) {
  //  std:: cout<< "Left =" << leftPower;
  //  std:: cout<< "Right =" << rightPower << std::endl;
   if (frontIsForward) {
      leftFront.Set(leftPower * powerScaling);
      leftRear.Set(leftPower * powerScaling);
      rightFront.Set(rightPower * powerScaling);
      rightRear.Set(rightPower * powerScaling);
  } else {
      leftFront.Set(-rightPower * powerScaling);
      leftRear.Set(-rightPower * powerScaling);
      rightFront.Set(-leftPower * powerScaling);
      rightRear.Set(-leftPower * powerScaling);
  }
}