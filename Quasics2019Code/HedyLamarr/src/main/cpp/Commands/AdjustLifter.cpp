/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "Commands/AdjustLifter.h"
#include "OI.h"
#include "Robot.h"
#include "Subsystems/Lifter.h"

AdjustLifter::AdjustLifter() {
  Requires(Robot::lifter.get());
}

// Called just before this Command runs the first time
void AdjustLifter::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void AdjustLifter::Execute() {
  if(Robot::oi->isMoveUpSignaled() && !Robot::lifter->atTop()){
    Robot::elevator->moveSlowlyUp();
    moving_up = true;
    moving_down = false;
  }
  else if(Robot::oi->isMoveDownSignaled()&& !Robot::lifter->atBottom()){
    Robot::lifter->moveSlowlyDown();
    moving_up = false;
    moving_down = true;
  }
  else{
    Robot::lifter->stop();
    moving_up = false;
    moving_down = false;
  }
}

// Make this return true when this Command no longer needs to run execute()
bool AdjustLifter::IsFinished() {
return false;
}

// Called once after isFinished returns true
void AdjustLifter::End() {
  Robot::lifter->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AdjustLifter::Interrupted() {
  Robot::lifter->stop();
}
