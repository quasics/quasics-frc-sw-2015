/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/ElevatorToTop.h"
#include "Robot.h"
#include "Subsystems/Elevator.h"

ElevatorToTop::ElevatorToTop() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::elevator.get());
}

// Called just before this Command runs the first time
void ElevatorToTop::Initialize() {
  if (Robot::elevator->atTop()) {
    // better to have a stop and not need it (even if we shouldn't ever need it)
    Robot::elevator->stop();
  } else {
    Robot::elevator->moveUp();
  }
}

// Called repeatedly when this Command is scheduled to run
void ElevatorToTop::Execute() {
}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorToTop::IsFinished() {
  if (Robot::elevator->atTop()) {
    return true;
  } else {
    return false;
  }
}

// Called once after isFinished returns true
void ElevatorToTop::End() {
  Robot::elevator->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElevatorToTop::Interrupted() {
  End();
}
