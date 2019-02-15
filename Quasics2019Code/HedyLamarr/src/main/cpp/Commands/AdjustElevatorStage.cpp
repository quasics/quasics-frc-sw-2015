/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/AdjustElevatorStage.h"

#include "Robot.h"

AdjustElevatorSingleStage::AdjustElevatorSingleStage(ElevatorStage& stage)
    : stage(stage) {
  Requires(&stage);
}

// Called repeatedly when this Command is scheduled to run
void AdjustElevatorSingleStage::Execute() {
  // tests whether the elevator is signaled to go up, and if it isn't at the top
  if (Robot::oi->isElevatorMoveUpSignaled() && !stage.atTop()) {
    // continues if both states are true
    stage.moveSlowlyUp();
  }
  // tests whether the elevator is signaled to go up, and if it isn't at the top
  else if (Robot::oi->isElevatorMoveDownSignaled() && !stage.atBottom()) {
    // continues if both states are true
    stage.moveSlowlyDown();
  } else {
    // if neither is true, it stops
    stage.stop();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool AdjustElevatorSingleStage::IsFinished() {
  // this command is default for the elevator and never stops, it is only
  // interrupted
  return false;
}

// Called once after isFinished returns true
void AdjustElevatorSingleStage::End() {
  // putting in an end state to stop because I'm paranoid
  stage.stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AdjustElevatorSingleStage::Interrupted() {
  End();
}
