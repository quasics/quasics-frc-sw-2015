/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/NewElevatorToLow.h"
#include "Robot.h"
#include "Subsystems/NewElevator.h"

NewElevatorToLow::NewElevatorToLow() {
  // Use Requires() here to declare subsystem dependencies
  Requires(Robot::newElevator.get());
}

// Called just before this Command runs the first time
void NewElevatorToLow::Initialize() {
  if(Robot::newElevator->atLow()){
    Robot::newElevator->stop();
    moving_down = false;
  }
  else if(Robot::newElevator->atBottom()){
    Robot::newElevator->move(.6);
    moving_down = false;
  }
  else{
    Robot::newElevator->move(-.6);
    moving_down = false;
  }
}

// Called repeatedly when this Command is scheduled to run
void NewElevatorToLow::Execute() {
  if(needs_to_switch){
    if(moving_down){
      Robot::newElevator->move(.6);
      moving_down = false;
      needs_to_switch = false;
    }
    else{
      Robot::newElevator->move(-.6);
      moving_down = true;
      needs_to_switch = false;
    }
  }
}

// Make this return true when this Command no longer needs to run execute()
bool NewElevatorToLow::IsFinished() { 
  if(Robot::newElevator->atLow()){
    return true;
  }
  if(Robot::newElevator->atBottom() && moving_down){
    needs_to_switch = true;
  }
  else if(Robot::newElevator->atTop() && !moving_down){
    needs_to_switch = true;
  }
  else if(Robot::newElevator->atMedium() && !moving_down){
    needs_to_switch = true;
  }
  return false;
}

// Called once after isFinished returns true
void NewElevatorToLow::End() {
  Robot::newElevator->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void NewElevatorToLow::Interrupted() {
  End();
}
