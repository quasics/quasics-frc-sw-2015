/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/ElbowControl.h"

ElbowControl::ElbowControl() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void ElbowControl::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ElbowControl::Execute() {
  if (Robot::oi->OI::isElbowSignaledUp()) {
    Robot::elbow->ElbowUp();
  } else if (Robot::oi->OI::isElbowSignaledDown()) {
    Robot::elbow->ElbowDown();
  } else {
    Robot::elbow->Stop();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool ElbowControl::IsFinished() {
  return false;
}

// Called once after isFinished returns true
void ElbowControl::End() {
  Robot::elbow->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElbowControl::Interrupted() {
  End();
}
