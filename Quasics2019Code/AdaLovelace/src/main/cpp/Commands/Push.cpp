/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Push.h"
#include "Robot.h"

Push::Push() {
  // Use Requires() here to declare subsystem dependencies
  Requires(Robot::pusher.get());
}

// Called just before this Command runs the first time
void Push::Initialize() {
  Robot::pusher->SetPusherExtension(490);
}

// Called repeatedly when this Command is scheduled to run
void Push::Execute() {

  if(Robot::oi->ShouldPush()){
    Robot::pusher->SetPusherExtension(2000);
  }
  else{
    Robot::pusher->SetPusherExtension(490);
    //490
  }
}

// Make this return true when this Command no longer needs to run execute()
bool Push::IsFinished() { return false; }

// Called once after isFinished returns true
void Push::End() {
  Robot::pusher->SetPusherExtension(490);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Push::Interrupted() {
  End();
}
