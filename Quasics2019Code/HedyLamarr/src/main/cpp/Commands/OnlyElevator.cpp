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
  auto& liftingBody = Robot::elevator;
  if (Robot::oi->isElevatorMoveUpSignaled()) {
    if (!liftingBody->atTop()) {
      std::cerr << "Telling elevator to move up\n";
      liftingBody->moveSlowlyUp();
    } else {
      std::cerr
          << "Cowardly refusing to move Elevator up, since we're at the top\n";
      liftingBody->stop();
    }
  } else if (Robot::oi->isElevatorMoveDownSignaled()) {
    if (!liftingBody->atBottom()) {
      std::cerr << "Telling elevator to move down\n";
      liftingBody->moveSlowlyDown();
    } else {
      std::cerr << "Cowardly refusing to move Elevator down, since we're at "
                   "the bottom\n";
      liftingBody->stop();
    }
  } else {
    // if neither is true, it stops
    std::cerr << "Stopping elevator\n";
    liftingBody->stop();
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
