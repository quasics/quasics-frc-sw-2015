/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/MoveForDuration.h"
#include "Robot.h"

MoveForDuration::MoveForDuration(double timeout)
    : TimedCommand(timeout) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::driveBase.get());
}

// Called just before this Command runs the first time
void MoveForDuration::Initialize() {
  Robot::driveBase->SetPowerToMotors(0.25, 0.25);
}

// Called repeatedly when this Command is scheduled to run
void MoveForDuration::Execute() {}

// Called once after command times out
void MoveForDuration::End() {
Robot::driveBase->Stop();  
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForDuration::Interrupted() {
  End();
}

