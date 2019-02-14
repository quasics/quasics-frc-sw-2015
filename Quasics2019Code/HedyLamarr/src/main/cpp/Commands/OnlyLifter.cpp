/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/OnlyLifter.h"

#include <iostream>
#include "OI.h"
#include "Robot.h"
#include "Subsystems/Lifter.h"

OnlyLifter::OnlyLifter() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::lifter.get());
}

// Called just before this Command runs the first time
void OnlyLifter::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void OnlyLifter::Execute() {
  if (Robot::oi->isElevatorMoveUpSignaled()) {
    if (!Robot::lifter->atTop()) {
      Robot::lifter->moveSlowlyUp();
    } else {
      std::cerr
          << "Cowardly refusing to move lifter up, since we're at the top\n";
    }
  }
  // tests whether the lifter is signaled to go down, and if it isn't at the
  // bottom
  else if (Robot::oi->isElevatorMoveDownSignaled()) {
    if (!Robot::lifter->atBottom()) {
      Robot::lifter->moveSlowlyDown();
    } else {
      std::cerr << "Cowardly refusing to move lifter down, since we're at the "
                   "bottom\n";
      Robot::lifter->stop();
    }
  } else {
    // if neither is true, it stops
    Robot::lifter->stop();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool OnlyLifter::IsFinished() {
  return false;
}

// Called once after isFinished returns true
void OnlyLifter::End() {
  Robot::lifter->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void OnlyLifter::Interrupted() {
  End();
}
