/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/NewHatchPanelManipulatorControl.h"

NewHatchPanelManipulatorControl::NewHatchPanelManipulatorControl() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}


// Called repeatedly when this Command is scheduled to run
void NewHatchPanelManipulatorControl::Execute() {
  if (Robot::oi->OI::isHatchManipulatorSignaledOpen()) {
    Robot::newHatchPanelManipulator->SetPower(.9);
  } else if (Robot::oi->OI::isHatchManipulatorSignaledClose()) {
    Robot::newHatchPanelManipulator->SetPower(-.9);
  } else {
    Robot::newHatchPanelManipulator->SetPower(0);
  }
}

// Make this return true when this Command no longer needs to run execute()
bool NewHatchPanelManipulatorControl::IsFinished() { return false; }

// Called once after isFinished returns true
void NewHatchPanelManipulatorControl::End() {
  Robot::newHatchPanelManipulator->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void NewHatchPanelManipulatorControl::Interrupted() {
  End();
}
