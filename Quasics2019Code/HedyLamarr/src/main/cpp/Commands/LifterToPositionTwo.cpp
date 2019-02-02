/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "Subsystems/Lifter.h"
#include "Commands/LifterToPositionTwo.h"
#include "Robot.h"

LifterToPositionTwo::LifterToPositionTwo() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::lifter.get());
}

// Called just before this Command runs the first time
void LifterToPositionTwo::Initialize() {
   if(Robot::lifter->atPositionTwo()){
    Robot::lifter->stop();
  }
  else if(Robot::lifter->atTop()){
    Robot::lifter->moveDown();
  }
  else{
    Robot::lifter->moveUp();
  }
}

// Called repeatedly when this Command is scheduled to run
void LifterToPositionTwo::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool LifterToPositionTwo::IsFinished() {
  if(Robot::lifter->atPositionTwo()){
    return true;
  }
  else{
    return false;
  }
 }

// Called once after isFinished returns true
void LifterToPositionTwo::End() {
   Robot::lifter->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LifterToPositionTwo::Interrupted() {
  End();
}
