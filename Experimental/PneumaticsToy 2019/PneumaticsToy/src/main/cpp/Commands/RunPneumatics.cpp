/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/RunPneumatics.h"
#include "Robot.h"
#include "OI.h"

RunPneumatics::RunPneumatics() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::pneumatics.get());
}

// Called just before this Command runs the first time
void RunPneumatics::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunPneumatics::Execute() {
  if(Robot::oi->OI::IsForwardSignaled()){
    Robot::pneumatics->SetSolenoidForward();
  }
  else if(Robot::oi->OI::IsReverseSignaled()){
    Robot::pneumatics->SetSolenoidReverse();
  }
  else{
    Robot::pneumatics->SetSolenoidOff();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool RunPneumatics::IsFinished() { return false; }

// Called once after isFinished returns true
void RunPneumatics::End() {
  Robot::pneumatics->SetSolenoidOff();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void RunPneumatics::Interrupted() {
  End();
}
