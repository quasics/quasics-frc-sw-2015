#include "MoveAheadFast.h"
#include "../Robot.h"

MoveAheadFast::MoveAheadFast()
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
	Requires(Robot::driveTrain.get());
}

// Called just before this Command runs the first time
void MoveAheadFast::Initialize()
{
Robot::driveTrain->SetLeftPower(.75);
Robot::driveTrain->SetRightPower(.75);
timesMoved=0;
}

// Called repeatedly when this Command is scheduled to run
void MoveAheadFast::Execute()
{
timesMoved=timesMoved+1;
}

// Make this return true when this Command no longer needs to run execute()
bool MoveAheadFast::IsFinished()
{
	if(timesMoved>=25)return true;
	return false;
}

// Called once after isFinished returns true
void MoveAheadFast::End()
{
Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveAheadFast::Interrupted()
{
Robot::driveTrain->Stop();
}
