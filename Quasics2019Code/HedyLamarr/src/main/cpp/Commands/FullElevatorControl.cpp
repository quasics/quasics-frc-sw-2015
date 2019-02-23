/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/FullElevatorControl.h"

#include "Robot.h"
#include <iostream>

FullElevatorControl::FullElevatorControl() {
  // Use Requires() here to declare subsystem dependencies
  Requires(Robot::lifter.get());
  Requires(Robot::elevator.get());
}

// Called just before this Command runs the first time
void FullElevatorControl::Initialize() {
  pressedLastTime = false;
  toggleCounter = 0;
}

// Called repeatedly when this Command is scheduled to run
void FullElevatorControl::Execute() {
  if(Robot::oi->isElevatorToggleDown() && !pressedLastTime){
    Robot::lifter->stop();
    Robot::elevator->stop();
    pressedLastTime = true;
    toggleCounter = (toggleCounter + 1) % 3;

  }
  else if(!Robot::oi->isElevatorToggleDown()){
    pressedLastTime = false;
  }

  if(toggleCounter == 0){
    //move both stages at once
    adjustElevatorStage(*Robot::elevator);
    adjustElevatorStage(*Robot::lifter);
    Robot::SetLifterMode(Robot::eBoth);
  }
  else if(toggleCounter == 1){
    //move lifter only
    adjustElevatorStage(*Robot::lifter);
    Robot::SetLifterMode(Robot::eLifter);
  }
  else if(toggleCounter == 2){
    //move elevator only
    adjustElevatorStage(*Robot::elevator);
    Robot::SetLifterMode(Robot::eElevator);
  }
}

#undef SIMULATE_MOTION
#undef NOISY

#ifdef NOISY
  #define LOG(x)  std::cerr << x
#else
  #define LOG(x)  
#endif

void FullElevatorControl::adjustElevatorStage(ElevatorStage & stage) {
  // tests whether the elevator is signaled to go up, and if it isn't at the top
  if (Robot::oi->isElevatorMoveUpSignaled() && !stage.atTop()) {
    // continues if both states are true
    LOG("Start moving " << stage.GetName() << " up\n");
#ifndef SIMULATE_MOTION
    stage.moveSlowlyUp();
#endif
  }
  // tests whether the elevator is signaled to go up, and if it isn't at the top
  else if (Robot::oi->isElevatorMoveDownSignaled() && !stage.atBottom()) {
    // continues if both states are true
    LOG("Start moving " << stage.GetName() << " down\n");
#ifndef SIMULATE_MOTION
    stage.moveSlowlyDown();
#endif
  } else {
    // if neither is true, it stops
    LOG("Stop moving " << stage.GetName() << "\n");
#ifndef SIMULATE_MOTION
    stage.stop();
#endif
  }
}

// Make this return true when this Command no longer needs to run execute()
bool FullElevatorControl::IsFinished() { return false; }

// Called once after isFinished returns true
void FullElevatorControl::End() {
  Robot::lifter->stop();
  Robot::elevator->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FullElevatorControl::Interrupted() {
  LOG("******** Full elevator control was interrupted!\n");
  End();
}
