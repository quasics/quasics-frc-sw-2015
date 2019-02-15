/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/ElevatorSingleStageTest.h"

#include <iostream>
#include "Robot.h"

// Called repeatedly when this Command is scheduled to run
void ElevatorSingleStageTest::Execute() {
  if (Robot::oi->isElevatorMoveUpSignaled()) {
    if (!stage.atTop()) {
      std::cerr << "Telling " << stage.GetName() << " to move up\n";
      stage.moveSlowlyUp();
    } else {
      std::cerr << "Cowardly refusing to move " << stage.GetName()
                << " up, since we're at the top\n";
      stage.stop();
    }
  } else if (Robot::oi->isElevatorMoveDownSignaled()) {
    if (!stage.atBottom()) {
      std::cerr << "Telling " << stage.GetName() << " to move down\n";
      stage.moveSlowlyDown();
    } else {
      std::cerr << "Cowardly refusing to move " << stage.GetName()
                << " down, since we're at "
                   "the bottom\n";
      stage.stop();
    }
  } else {
    // if neither is true, it stops
    std::cerr << "Stopping " << stage.GetName() << "\n";
    stage.stop();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorSingleStageTest::IsFinished() {
  return false;
}

// Called once after isFinished returns true
void ElevatorSingleStageTest::End() {
  stage.stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElevatorSingleStageTest::Interrupted() {
  End();
}
