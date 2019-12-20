/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TestNikeDriveBaseMotor.h"

#include "Robot.h"

TestNikeDriveBaseMotor::TestNikeDriveBaseMotor(NikeDriveBase::Motor motor,
                                               double speed, double timeout)
    : TimedCommand(timeout), motor(motor), speed(speed) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_nikeDriveBase);
}

// Called just before this Command runs the first time
void TestNikeDriveBaseMotor::Initialize() {
  Robot::m_nikeDriveBase.SetSingleMotorPower(motor, speed);
}

// Called repeatedly when this Command is scheduled to run
void TestNikeDriveBaseMotor::Execute() {}

// Called once after command times out
void TestNikeDriveBaseMotor::End() {
  Robot::m_nikeDriveBase.SetSingleMotorPower(motor, 0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TestNikeDriveBaseMotor::Interrupted() { End(); }
