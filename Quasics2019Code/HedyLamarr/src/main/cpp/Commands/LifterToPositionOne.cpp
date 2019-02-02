/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "Commands/LifterToPositionOne.h"
#include "Robot.h"
#include "Subsystems/Lifter.h"

LifterToPositionOne::LifterToPositionOne() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::lifter.get());
}

// Called just before this Command runs the first time
void LifterToPositionOne::Initialize() {
  if (Robot::lifter->atPositionOne()) {
    Robot::lifter->stop();
    moving_down = false;
  } else if (Robot::lifter->atBottom()) {
    Robot::lifter->moveUp();
    moving_down = false;
  } else if (Robot::lifter->atPositionTwo()) {
    Robot::lifter->moveDown();
    moving_down = true;
  } else if (Robot::lifter->atTop()) {
    Robot::lifter->moveDown();
    moving_down = true;
  } else {
    Robot::lifter->moveDown();
    moving_down = true;
  }
}

// Called repeatedly when this Command is scheduled to run
void LifterToPositionOne::Execute() {
  if (needs_to_switch) {
    if (moving_down) {
      Robot::lifter->moveUp();
      moving_down = false;
    } else {
      Robot::lifter->moveDown();
      moving_down = true;
    }
    needs_to_switch = false;
  }
}

// Make this return true when this Command no longer needs to run execute()
bool LifterToPositionOne::IsFinished() {
  // Are we where we want to be?
  if (Robot::lifter->atPositionOne()) {
    return true;
  }

  // if we aren't, do we need to move differently?
  if (Robot::lifter->atBottom() && moving_down) {
    needs_to_switch = true;
  } else if (Robot::lifter->atTop() && !moving_down) {
    needs_to_switch = true;
  } else if (Robot::lifter->atPositionTwo() && !moving_down) {
    needs_to_switch = true;
  }

  // just keep swimmin'
  return false;
}

// Called once after isFinished returns true
void LifterToPositionOne::End() {
  Robot::lifter->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LifterToPositionOne::Interrupted() {
  End();
}
