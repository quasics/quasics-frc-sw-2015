/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/NikeDriveBase.h"

constexpr int LEFT_FRONT_ID = 0;
constexpr int LEFT_REAR_ID = 1;
constexpr int RIGHT_FRONT_ID = 2;
constexpr int RIGHT_REAR_ID = 3;

NikeDriveBase::NikeDriveBase()
    : Subsystem("NikeDriveBase"),
      leftFront(LEFT_FRONT_ID, rev::CANSparkMax::MotorType::kBrushless),
      leftRear(LEFT_REAR_ID, rev::CANSparkMax::MotorType::kBrushless),
      rightFront(RIGHT_FRONT_ID, rev::CANSparkMax::MotorType::kBrushless),
      rightRear(RIGHT_REAR_ID, rev::CANSparkMax::MotorType::kBrushless) {}

void NikeDriveBase::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
