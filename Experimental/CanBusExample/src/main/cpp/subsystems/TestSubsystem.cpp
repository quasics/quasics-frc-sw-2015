/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/TestSubsystem.h"
#include "RobotMap.h"

TestSubsystem::TestSubsystem()
    : Subsystem("ExampleSubsystem"),
      motorController(new TalonSRX{kTestMotorCantroller_CanId})
{
  motorController->Set(ControlMode::PercentOutput, 0);
}

void TestSubsystem::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
void TestSubsystem::setMotorPower(double percent) {
  motorController->Set(ControlMode::PercentOutput, percent);
}
