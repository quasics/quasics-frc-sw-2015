// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "StubCommand2.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

StubCommand2::StubCommand2() :
		Command() {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::driveSystem.get());
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	isFinished = false;
	duodecagon = kstop;
	timer = 0;
	Robot::driveSystem->GetContinuousYaw();

}
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void StubCommand2::Initialize() {
	isFinished = false;
	duodecagon = kstop;
	timer = 0;
	Robot::driveSystem->ResetYaw();

}

// Called repeatedly when this Command is scheduled to run
void StubCommand2::Execute() {
	if (timer <= 50) {
		duodecagon = kforward;
	} else if (Robot::driveSystem->GetContinuousYaw() >= -324) {
		duodecagon = kturnright;
		timer = 0;
	} else if (timer <= 50) {
		duodecagon = kforward;
		Robot::driveSystem->ResetYaw();
	} else if (Robot::driveSystem->GetContinuousYaw() >= -324) {
		duodecagon = kturnright;
		timer = 0;
	} else if (timer <= 50) {
		duodecagon = kforward;
		Robot::driveSystem->ResetYaw();
	} else if (Robot::driveSystem->GetContinuousYaw() >= -324) {
		duodecagon = kturnright;
		timer = 0;
	} else if (timer <= 50) {
		duodecagon = kforward;
		Robot::driveSystem->ResetYaw();
	} else if (Robot::driveSystem->GetContinuousYaw() >= -324) {
		duodecagon = kturnright;
		timer = 0;
	} else if (timer <= 50) {
		duodecagon = kforward;
		Robot::driveSystem->ResetYaw();
	} else if (Robot::driveSystem->GetContinuousYaw() >= -324) {
		duodecagon = kturnright;
		timer = 0;
	}else {
		duodecagon = kstop;
		isFinished = true;
	}

	if (duodecagon == kforward) {
		Robot::driveSystem->MoveLeft(30);
		Robot::driveSystem->MoveRight(30);

	} else if (duodecagon == kturnright) {
		Robot::driveSystem->MoveLeft(40);
		Robot::driveSystem->MoveRight(-40);
	} else {
		Robot::driveSystem->StopEverything();
	}

	timer++;
}
// Make this return true when this Command no longer needs to run execute()
bool StubCommand2::IsFinished() {
	return isFinished;
}

// Called once after isFinished returns true
void StubCommand2::End() {
	Robot::driveSystem->StopEverything();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void StubCommand2::Interrupted() {
	Robot::driveSystem->StopEverything();

}
