/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/ElevatorToBottom.h"
#include "Subsystems/Elevator.h"
#include "Robot.h"

ElevatorToBottom::ElevatorToBottom() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::elevator.get());
}

// Called just before this Command runs the first time
void ElevatorToBottom::Initialize() {
  if(Robot::elevator->atBottom()){
    Robot::elevator->stop();
  }
  else{
    Robot::elevator->moveDown();
  }
}

// Called repeatedly when this Command is scheduled to run
void ElevatorToBottom::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorToBottom::IsFinished() { 
  if(Robot::elevator->atBottom()){
    return true;
  }
  else{
    return false;
  }
}

// Called once after isFinished returns true
void ElevatorToBottom::End() {
  Robot::elevator->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElevatorToBottom::Interrupted() {
  End();
}
