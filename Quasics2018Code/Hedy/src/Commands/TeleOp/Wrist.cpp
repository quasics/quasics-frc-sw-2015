/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Wrist.h"
#include "../../Robot.h"
#include "../../ControllerDefinitions.h"
Wrist::Wrist() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::cubeIntake.get());
}

// Called just before this Command runs the first time
void Wrist::Initialize() {
	previousButton = false;
}

// Called repeatedly when this Command is scheduled to run
void Wrist::Execute() {
	if (previousButton && !Robot::oi->getauxStick()->GetRawButton(XBox_ButtonX)) {
		if (Robot::cubeIntake->Get()) {
			Robot::cubeIntake->Set(false);
		} else {
			Robot::cubeIntake->Set(true);
		}
	}
	previousButton = Robot::oi->getauxStick()->GetRawButton(XBox_ButtonX);
}

// Make this return true when this Command no longer needs to run execute()
bool Wrist::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void Wrist::End() {
	Robot::cubeIntake->Set(false);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Wrist::Interrupted() {
	End();
}
