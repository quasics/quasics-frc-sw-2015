// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
#include <iostream>
#include "AutonomousCommand.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

AutonomousCommand::AutonomousCommand(int TargetPosition) :
		Command() {
	m_TargetPosition = TargetPosition;
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	isEndstep = false;
	autoStage = 0;

	Robot::driveSystem->ResetEncoders();
	Robot::driveSystem->ResetYaw();
}

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void AutonomousCommand::Initialize() {
	isEndstep = false;
	autoStage = 0;

	Robot::driveSystem->ResetEncoders();
	Robot::driveSystem->ResetYaw();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCommand::Execute() {
	if (!isEndstep) {
		switch (m_TargetPosition) {
		case (1):
			if (Auto1())
				isEndstep = true;
			break;

		case (2):
			if (Auto2())
				isEndstep = true;
			break;

		case (3):
			if (Auto3())
				isEndstep = true;
			break;

		case (4):
			if (Auto4())
				isEndstep = true;
			break;

		case (5):
			if (Auto5())
				isEndstep = true;
			break;
		default:
			isEndstep = true;
			break;
		}
	} else {
		Robot::driveSystem->StopEverything();
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutonomousCommand::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void AutonomousCommand::End() {
	Robot::driveSystem->StopEverything();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutonomousCommand::Interrupted() {
	Robot::driveSystem->StopEverything();
}

