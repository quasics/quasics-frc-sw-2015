/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//getting all the files it needs
#include "Commands/AdjustLifter.h"
#include "OI.h"
#include "Robot.h"
#include "Subsystems/Lifter.h"

AdjustLifter::AdjustLifter() {
  //gets the lifter subsystem
  Requires(Robot::lifter.get());
}

// Called just before this Command runs the first time
void AdjustLifter::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void AdjustLifter::Execute() {
  //tests whether the lifter is signaled to go up, and if it isn't at the top
  if (Robot::oi->isElevatorMoveUpSignaled() && !Robot::lifter->atTop()) {
    //continues if both states are true
    Robot::lifter->moveSlowlyUp();
  }
  //tests whether the lifter is signaled to go down, and if it isn't at the bottom
   else if (Robot::oi->isElevatorMoveDownSignaled() &&!Robot::lifter->atBottom()) {
     //continues if both states are true
    Robot::lifter->moveSlowlyDown();
  } 
  else {
     //if neither is true, it stops
    Robot::lifter->stop();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool AdjustLifter::IsFinished() {
  //this command is default for the lifter and never stops, it is only interrupted
  return false;
}

// Called once after isFinished returns true
void AdjustLifter::End() {
  //putting in an end state to stop because I'm paranoid
  Robot::lifter->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AdjustLifter::Interrupted() {
  //when it's interrupted it needs to stop
  Robot::lifter->stop();
}
