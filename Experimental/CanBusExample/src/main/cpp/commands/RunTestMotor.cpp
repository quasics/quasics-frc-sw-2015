/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RunTestMotor.h"
#include "Robot.h"

RunTestMotor::RunTestMotor() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_testSubsystem);
}

// Called just before this Command runs the first time
void RunTestMotor::Initialize() {
  Robot::m_testSubsystem.setMotorPower(1);
}

// Called repeatedly when this Command is scheduled to run
void RunTestMotor::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool RunTestMotor::IsFinished() { return false; }

// Called once after isFinished returns true
void RunTestMotor::End() {
  Robot::m_testSubsystem.stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void RunTestMotor::Interrupted() {
  End();
}
