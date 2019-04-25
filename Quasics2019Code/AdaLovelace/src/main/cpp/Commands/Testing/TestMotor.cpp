/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Testing/TestMotor.h"

TestMotor::TestMotor(frc::Subsystem &subsystem, frc::SpeedController &speedController, double power)
    : speedController(speedController), power(power)
{
  Requires(&subsystem);
}

// Called just before this Command runs the first time
void TestMotor::Initialize() {
  speedController.Set(power);
}

// Called once after isFinished returns true
void TestMotor::End() {
  speedController.Set(0);
}

void TestMotor::Interrupted() { End(); }

bool TestMotor::IsFinished() { return false; }
