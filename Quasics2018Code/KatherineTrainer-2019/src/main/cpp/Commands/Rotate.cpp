/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Rotate.h"
#include "Robot.h"

Rotate::Rotate(double timeout)
    : TimedCommand(timeout) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void Rotate::Initialize() {
  Robot::driveBase->SetPowerToMotors(0.30, -0.30);//Make the robot turn
}

// Called repeatedly when this Command is scheduled to run
void Rotate::Execute() {}

// Called once after command times out
void Rotate::End() {
  Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Rotate::Interrupted() {
  End();
}
