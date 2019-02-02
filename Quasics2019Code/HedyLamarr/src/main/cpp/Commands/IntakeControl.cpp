/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/IntakeControl.h"

IntakeControl::IntakeControl() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called repeatedly when this Command is scheduled to run
void IntakeControl::Execute() {
  if (Robot::oi->OI::isIntakeSignaledPositive()) {
    Robot::manipulator->SetIntakePower(1);
  } else if (Robot::oi->OI::isIntakeSignaledHighNegative()) {
    Robot::manipulator->SetIntakePower(-1);
  } else if (Robot::oi->OI::isIntakeSignaledLowNegative()) {
    Robot::manipulator->SetIntakePower(-.5);
  } else {
    Robot::manipulator->SetIntakePower(0);
  }
}

// Make this return true when this Command no longer needs to run execute()
bool IntakeControl::IsFinished() {
  return false;
}

// Called once after isFinished returns true
void IntakeControl::End() {
  Robot::manipulator->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void IntakeControl::Interrupted() {
  End();
}
