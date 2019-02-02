/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/ElevatorToPositionOne.h"
#include "Robot.h"
#include "Subsystems/Elevator.h"

ElevatorToPositionOne::ElevatorToPositionOne() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::elevator.get());
}

// Called just before this Command runs the first time
void ElevatorToPositionOne::Initialize() {
  if (Robot::elevator->atPositionOne()) {
    Robot::elevator->stop();
    moving_down = false;
  } 
  else if (Robot::elevator->atBottom()) {
    Robot::elevator->moveUp();
    moving_down = false;
  } 
  else if(Robot::elevator->atPositionTwo()){
    Robot::elevator->moveDown();
    moving_down = true;
  }
  else if(Robot::elevator->atTop()){
    Robot::elevator->moveDown();
    moving_down = true;
  }
  else{
    Robot::elevator->moveDown();
    moving_down = true;
  }
}

// Called repeatedly when this Command is scheduled to run
void ElevatorToPositionOne::Execute() {
  if (needs_to_switch){
    if(moving_down){
      Robot::elevator->moveUp();
      moving_down = false;
    }
    else{
      Robot::elevator->moveDown();
      moving_down = true;
    }
  }
}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorToPositionOne::IsFinished() {
  //see if we're here
  if (Robot::elevator->atPositionOne()) {
    return true;
  }

  //if we aren't, do we need to move differently?
  if(Robot::elevator->atBottom()&& moving_down) {
    needs_to_switch = true;
  }
  else if(Robot::elevator->atTop()&& !moving_down){
    needs_to_switch = true;
  }
  else if(Robot::elevator->atPositionTwo()&& !moving_down){
    needs_to_switch = true;
  }

  //just keep swimmin'
  return false;
}

// Called once after isFinished returns true
void ElevatorToPositionOne::End() {
  Robot::elevator->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElevatorToPositionOne::Interrupted() {
  End();
}
