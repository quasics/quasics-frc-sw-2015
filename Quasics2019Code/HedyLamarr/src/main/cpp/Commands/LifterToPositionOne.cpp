/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "Subsystems/Lifter.h"
#include "Commands/LifterToPositionOne.h"
#include "Robot.h"

LifterToPositionOne::LifterToPositionOne() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
   Requires(Robot::lifter.get());
}

// Called just before this Command runs the first time
void LifterToPositionOne::Initialize() {
   if(Robot::lifter->atPositionOne()){
    Robot::lifter->stop();
  }
  else if(Robot::lifter->atBottom()){
    Robot::lifter->moveUp();
  }
  else{
    Robot::lifter->moveDown();
  }
}

// Called repeatedly when this Command is scheduled to run
void LifterToPositionOne::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool LifterToPositionOne::IsFinished() { 
  if(Robot::lifter->atPositionOne()){
    return true;
  }
  else{
    return false;
  }
}

// Called once after isFinished returns true
void LifterToPositionOne::End() {
   Robot::lifter->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LifterToPositionOne::Interrupted() {
  End();
}
