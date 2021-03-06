#include "TurnLeftContinuous.h"
#include "../Robot.h"

TurnLeftContinuous::TurnLeftContinuous()
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::driveBase.get());
}

// Called just before this Command runs the first time
void TurnLeftContinuous::Initialize() {
	Robot::driveBase->SetPower(-.25, .25);
}

// Called repeatedly when this Command is scheduled to run
void TurnLeftContinuous::Execute() {
	// There's nothing to do during execution: the Initialize() code starts
	// the wheels turning.
}

// Make this return true when this Command no longer needs to run execute()
bool TurnLeftContinuous::IsFinished() {
	// We just want to keep running until something outside tells us to stop.
	return false;
}

// Called once after isFinished returns true
void TurnLeftContinuous::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TurnLeftContinuous::Interrupted() {
	Robot::driveBase->Stop();
}
