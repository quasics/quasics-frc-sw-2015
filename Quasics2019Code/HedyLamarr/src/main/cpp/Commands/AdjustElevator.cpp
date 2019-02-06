/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/AdjustElevator.h"
#include "OI.h"
#include "Robot.h"
#include "Subsystems/Elevator.h"

AdjustElevator::AdjustElevator() {
  Requires(Robot::elevator.get());
}

// Called just before this Command runs the first time
void AdjustElevator::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void AdjustElevator::Execute() {
  if (Robot::oi->isElevatorMoveUpSignaled() && !Robot::lifter->atTop()) {
    Robot::elevator->moveSlowlyUp();
  } else if (Robot::oi->isElevatorMoveDownSignaled() &&
             !Robot::lifter->atBottom()) {
    Robot::elevator->moveSlowlyDown();
  } else {
    Robot::lifter->stop();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool AdjustElevator::IsFinished() {
  return false;
}

// Called once after isFinished returns true
void AdjustElevator::End() {
  Robot::elevator->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AdjustElevator::Interrupted() {
  Robot::elevator->stop();
}
