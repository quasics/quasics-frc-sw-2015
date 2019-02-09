/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/LinkageControl.h"
#include "Robot.h"
#include "OI.h"
#include "Subsystems/TwoBarLinkage.h"
LinkageControl::LinkageControl() {
  // Use Requires() here to declare subsystem dependencies
  Requires(Robot::twoBarLinkage.get());
}

// Called just before this Command runs the first time
void LinkageControl::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LinkageControl::Execute() {
  if(Robot::oi->OI::isElevatorMoveUpSignaled()){
    Robot::twoBarLinkage->LinkageUp();
  }
  else if(Robot::oi->OI::isElevatorMoveDownSignaled()){
    Robot::twoBarLinkage->LinkageDown();
  }
  else{
    Robot::twoBarLinkage->Stop();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool LinkageControl::IsFinished() { 
  return false;
}

// Called once after isFinished returns true
void LinkageControl::End() {
  Robot::twoBarLinkage->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LinkageControl::Interrupted() {
  Robot::twoBarLinkage->Stop();
}
