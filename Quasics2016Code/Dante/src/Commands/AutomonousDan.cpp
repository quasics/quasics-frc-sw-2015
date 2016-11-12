#include "AutomonousDan.h"
#include "../Robot.h"
AutomonousDan::AutomonousDan()
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
	Requires(Robot::driveTrain.get());
}

// Called just before this Command runs the first time
void AutomonousDan::Initialize()
{
Robot::driveTrain->SetLeftPower(.6);
Robot::driveTrain->SetRightPower(.8);
timesMoved=0;


}

// Called repeatedly when this Command is scheduled to run
void AutomonousDan::Execute()
{
 timesMoved=timesMoved+1;
}

// Make this return true when this Command no longer needs to run execute()
bool AutomonousDan::IsFinished()
{
	return false;
}

// Called once after isFinished returns true
void AutomonousDan::End()
{
	Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutomonousDan::Interrupted()
{
Robot::driveTrain->Stop();
}
