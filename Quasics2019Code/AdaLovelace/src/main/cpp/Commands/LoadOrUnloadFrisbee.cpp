/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/LoadOrUnloadFrisbee.h"
#include "Robot.h"

LoadOrUnloadFrisbee::LoadOrUnloadFrisbee() {
  // Use Requires() here to declare subsystem dependencies
  Requires(Robot::loader.get());
}

// Called just before this Command runs the first time
void LoadOrUnloadFrisbee::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LoadOrUnloadFrisbee::Execute() {
  if(Robot::oi->ShouldLoad()){
    Robot::loader->SetLoaderMotors(.5);
  }
  if(Robot::oi->ShouldUnload()){
    Robot::loader->SetLoaderMotors(-.5);
  }
  else{
    Robot::loader->SetLoaderMotors(0);
  }
}

// Make this return true when this Command no longer needs to run execute()
bool LoadOrUnloadFrisbee::IsFinished() { return false; }

// Called once after isFinished returns true
void LoadOrUnloadFrisbee::End() {
  Robot::loader->SetLoaderMotors(0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LoadOrUnloadFrisbee::Interrupted() {
  End();
}
