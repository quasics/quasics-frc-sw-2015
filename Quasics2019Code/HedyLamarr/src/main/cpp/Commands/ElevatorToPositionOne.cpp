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
  } else if (Robot::elevator->atBottom()) {
    Robot::elevator->moveUp();
  } else {
    // CODE_REVIEW (mjh): I think you're making an assumption here that the
    // elevator will always be stopped at one of the defined positions.  If so,
    // you should document it.

    // CODE_REVIEW (mjh): I think you're making some assumptions here about the
    // relative positions (in terms of height) for stops 1&2.  This is probably
    // OK, but would be good to document.
    Robot::elevator->moveDown();

    // CODE_REVIEW (mh): What if the elevator isn't at position 2?  What if the
    // robot was stopped between the bottom and position 1?  How are you going
    // to handle that?  (More immediately, what's going to happen in this case
    // with the current code?)
  }
}

// Called repeatedly when this Command is scheduled to run
void ElevatorToPositionOne::Execute() {
}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorToPositionOne::IsFinished() {
  if (Robot::elevator->atPositionOne()) {
    // CODE_REVIEW (mjh): You need to handle the case where we "overrun" a
    // position, and don't sense it.  To be specific, what if we hit the
    // top/bottom sensor?  You've got nothing in here to stop us from
    // overrunning the elevator, and the h/w team is expecting the "hard stops"
    // to be implemented in s/w.
    return true;
  } else {
    return false;
  }
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
