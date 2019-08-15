/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/NewElevatorToTop.h"
#include "Robot.h"
#include "Subsystems/NewElevator.h"

NewElevatorToTop::NewElevatorToTop() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::newElevator.get());
}

// Called just before this Command runs the first time
void NewElevatorToTop::Initialize() {
  if(Robot::newElevator->atTop()) {
    Robot::newElevator->stop();
  }
  else {
    Robot::newElevator->move(0.6);
  }
}

// Called repeatedly when this Command is scheduled to run
void NewElevatorToTop::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool NewElevatorToTop::IsFinished() { 
  if(Robot::newElevator->atTop()) {
    return true;
  }
  return false;
}

// Called once after isFinished returns true
void NewElevatorToTop::End() {
  Robot::newElevator->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void NewElevatorToTop::Interrupted() {
  End();
}
