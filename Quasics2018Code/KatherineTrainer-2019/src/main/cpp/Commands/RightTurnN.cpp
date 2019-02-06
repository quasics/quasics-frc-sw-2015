/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/RightTurnN.h"
#include "robot.h"

//Nurfadil's Right Turn

RightTurnN::RightTurnN(double timeout)
    : TimedCommand(timeout) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void RightTurnN::Initialize() {
  Robot::driveBase->SetPowerToMotors(-0.25, 0.25);
}

// Called repeatedly when this Command is scheduled to run
void RightTurnN::Execute() {}

// Called once after command times out
void RightTurnN::End() {
  Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void RightTurnN::Interrupted() {
  End();
}
