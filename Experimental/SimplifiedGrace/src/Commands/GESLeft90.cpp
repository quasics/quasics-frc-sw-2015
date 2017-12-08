#include "GESLeft90.h"
#include "../Robot.h"

GESLeft90::GESLeft90()
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::driveBase.get());
	Requires(Robot::navigation.get());
}

// Called just before this Command runs the first time
void GESLeft90::Initialize() {
	Robot::driveBase->SetLeftPower(-.25);
	Robot::driveBase->SetRightPower(.25);
	Robot::navigation->resetBearing();
}

// Called repeatedly when this Command is scheduled to run
void GESLeft90::Execute() {
	// There's nothing to do during execution: the Initialize() code starts
	// the wheels turning.
}

// Make this return true when this Command no longer needs to run execute()
bool GESLeft90::IsFinished() {
	if (Robot::navigation->getBearing() > 89) {
		return true;
	}
	else {
		return false;
	}
}

// Called once after isFinished returns true
void GESLeft90::End() {
	Robot::driveBase->Stop();
	Robot::navigation->resetBearing();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GESLeft90::Interrupted() {
	Robot::driveBase->Stop();
	Robot::navigation->resetBearing();
}
