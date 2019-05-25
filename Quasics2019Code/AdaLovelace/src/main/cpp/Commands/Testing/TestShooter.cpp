/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Testing/TestShooter.h"

#include "Robot.h"

TestShooter::TestShooter(double speed) : speed(speed) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::shooter.get());
}

// Called just before this Command runs the first time
void TestShooter::Initialize() { Robot::shooter->SetShooterMotors(speed); }

// Called repeatedly when this Command is scheduled to run
void TestShooter::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool TestShooter::IsFinished() { return false; }

// Called once after isFinished returns true
void TestShooter::End() { Robot::shooter->SetShooterMotors(0); }

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TestShooter::Interrupted() { End(); }
