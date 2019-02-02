/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/ElevatorToPositionTwo.h"
#include "Subsystems/Elevator.h"
#include "Robot.h"

ElevatorToPositionTwo::ElevatorToPositionTwo() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::elevator.get());
}

// Called just before this Command runs the first time
void ElevatorToPositionTwo::Initialize() {
  if(Robot::elevator->atPositionTwo()){
    Robot::elevator->stop();
  }
  else if(Robot::elevator->atTop()){
    Robot::elevator->moveDown();
  }
  else{
    Robot::elevator->moveUp();
  }
}

// Called repeatedly when this Command is scheduled to run
void ElevatorToPositionTwo::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorToPositionTwo::IsFinished() {
  if(Robot::elevator->atPositionTwo()){
    return true;
  }
  else{
    return false;
  }
}

// Called once after isFinished returns true
void ElevatorToPositionTwo::End() {
  Robot::elevator->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElevatorToPositionTwo::Interrupted() {
  End();
}
