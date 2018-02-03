#include "FaceYellow.h"
#include "../Robot.h"

FaceYellow::FaceYellow() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::driveBase.get());
}

// Called just before this Command runs the first time
void FaceYellow::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void FaceYellow::Execute() {
	/*if (getImageRect)
	{

	}

	*/
}

// Make this return true when this Command no longer needs to run execute()
bool FaceYellow::IsFinished() {
	return false;

}

// Called once after isFinished returns true
void FaceYellow::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FaceYellow::Interrupted() {
	Robot::driveBase->Stop();
}
