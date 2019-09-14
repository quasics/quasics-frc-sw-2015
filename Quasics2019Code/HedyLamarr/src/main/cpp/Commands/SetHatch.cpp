/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/SetHatch.h"
#include "Robot.h"
#include "Subsystems/HatchPanelManipulator.h"
#include "OI.h"
#include <iostream>

SetHatch::SetHatch() {
  // Use Requires() here to declare subsystem dependencies
  Requires(Robot::hatchPanelManipulator.get());
}

// Called just before this Command runs the first time
void SetHatch::Initialize() {
  Robot::hatchPanelManipulator->MoveDown();
}

// Called repeatedly when this Command is scheduled to run
void SetHatch::Execute() {
  if(Robot::oi->isElbowSignaledUp()){
    std::cerr << "Signaled up" << std::endl;
      Robot::hatchPanelManipulator->MoveUp();
  }
  else if(Robot::oi->isElbowSignaledDown()){
    std::cerr << "Signaled down" << std::endl;
      Robot::hatchPanelManipulator->MoveDown();
  }
  else{
    Robot::hatchPanelManipulator->Stop();
    std::cerr << "The manipulator is currently stopped";
  }
}

// Make this return true when this Command no longer needs to run execute()
bool SetHatch::IsFinished() { return false; }

// Called once after isFinished returns true
void SetHatch::End() {
  Robot::hatchPanelManipulator->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetHatch::Interrupted() {
  End();
}
