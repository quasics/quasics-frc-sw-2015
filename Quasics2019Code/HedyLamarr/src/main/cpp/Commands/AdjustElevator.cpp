/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// getting all the files it needs
#include "Commands/AdjustElevator.h"
#include "OI.h"
#include "Robot.h"
#include "Subsystems/Elevator.h"

AdjustElevator::AdjustElevator() {
  // gets the elevator subsystem
  Requires(Robot::elevator.get());
}

// Called repeatedly when this Command is scheduled to run
void AdjustElevator::Execute() {
  // tests whether the elevator is signaled to go up, and if it isn't at the top
  if (Robot::oi->isElevatorMoveUpSignaled() && !Robot::lifter->atTop()) {
    // continues if both states are true
    Robot::elevator->moveSlowlyUp();
  }
  // tests whether the elevator is signaled to go up, and if it isn't at the top
  else if (Robot::oi->isElevatorMoveDownSignaled() &&
           !Robot::lifter->atBottom()) {
    // continues if both states are true
    Robot::elevator->moveSlowlyDown();
  } else {
    // if neither is true, it stops
    Robot::elevator->stop();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool AdjustElevator::IsFinished() {
  // this command is default for the elevator and never stops, it is only
  // interrupted
  return false;
}

// Called once after isFinished returns true
void AdjustElevator::End() {
  // putting in an end state to stop because I'm paranoid
  Robot::elevator->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AdjustElevator::Interrupted() {
  End();
}
