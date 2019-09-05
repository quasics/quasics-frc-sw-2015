/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/AdjustNewElevator.h"
#include "Subsystems/NewElevator.h"
#include "OI.h"
#include "Robot.h"
//#define ENABLE_NEW_SENSOR_CODE

AdjustNewElevator::AdjustNewElevator() {
  // Use Requires() here to declare subsystem dependencies
  Requires(Robot::newElevator.get());
}

// Called just before this Command runs the first time
void AdjustNewElevator::Initialize() {
  Robot::newElevator->stop();
}

// Called repeatedly when this Command is scheduled to run
void AdjustNewElevator::Execute() {
  if(Robot::oi->isElevatorMoveUpSignaled()){
    #ifdef ENABLE_NEW_SENSOR_CODE
    if(Robot::newElevator->atTop()){
      Robot::newElevator->stop();
    }
    else{
    #endif
      Robot::newElevator->move(.6);
    #ifdef ENABLE_NEW_SENSOR_CODE
    }
    #endif
  }
  else if(Robot::oi->isElevatorMoveDownSignaled()){
    #ifdef ENABLE_NEW_SENSOR_CODE
    if(Robot::newElevator->atBottom()){
      Robot::newElevator->stop();
    }
    else{
      #endif
      Robot::newElevator->move(-.6);
      #ifdef ENABLE_NEW_SENSOR_CODE
    }
    #endif
  }
  else{
    Robot::newElevator->stop();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool AdjustNewElevator::IsFinished() { 
  return false; 
}

// Called once after isFinished returns true
void AdjustNewElevator::End() {
  Robot::newElevator->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AdjustNewElevator::Interrupted() {
  End();
}
