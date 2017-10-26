#include "AutoTurnLeft.h"

AutoTurnLeft::AutoTurnLeft() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::driveBase.get());
}

// Called just before this Command runs the first time
void AutoTurnLeft::Initialize() {
	Robot::driveBase->SetLeftPower(power);
	Robot::driveBase->SetRightPower(power);
}

// Called repeatedly when this Command is scheduled to run
void AutoTurnLeft::Execute() {
	Robot::driveBase->SetLeftPower(.15);
	Robot::driveBase->SetRightPower(.15);
}

// Make this return true when this Command no longer needs to run execute()
bool AutoTurnLeft::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void AutoTurnLeft::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoTurnLeft::Interrupted() {
	Robot::driveBase->Stop();
}
