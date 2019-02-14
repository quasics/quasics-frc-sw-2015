/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/OnlyElevator.h"

#include <iostream>
#include "OI.h"
#include "Robot.h"
#include "Subsystems/Elevator.h"

OnlyElevator::OnlyElevator() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::elevator.get());
}

// Called just before this Command runs the first time
void OnlyElevator::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void OnlyElevator::Execute() {
  if (Robot::oi->isElevatorMoveUpSignaled()) {
    if (!Robot::elevator->atTop()) {
      std::cerr << "Telling elevator to move up\n";
      Robot::elevator->moveSlowlyUp();
    } else {
      std::cerr
          << "Cowardly refusing to move elevator up, since we're at the top\n";
      Robot::elevator->stop();
    }
  } else if (Robot::oi->isElevatorMoveDownSignaled()) {
    if (!Robot::elevator->atBottom()) {
      std::cerr << "Telling elevator to move down\n";
      Robot::elevator->moveSlowlyDown();
    } else {
      std::cerr << "Cowardly refusing to move elevator down, since we're at "
                   "the bottom\n";
      Robot::elevator->stop();
    }
  } else {
    // if neither is true, it stops
    std::cerr << "Stopping elevator\n";
    Robot::elevator->stop();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool OnlyElevator::IsFinished() {
  return false;
}

// Called once after isFinished returns true
void OnlyElevator::End() {
  Robot::elevator->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void OnlyElevator::Interrupted() {
  End();
}
