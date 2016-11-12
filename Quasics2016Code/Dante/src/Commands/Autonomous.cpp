#include "Autonomous.h"
#include "../Robot.h"

Autonomous::Autonomous()
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
	Requires(Robot::driveTrain.get());
}

// Called just before this Command runs the first time
void Autonomous::Initialize()
{
Robot:: driveTrain->SetLeftPower(.8);
Robot:: driveTrain->SetRightPower(.6);
timesMoved=0;
}

// Called repeatedly when this Command is scheduled to run
void Autonomous::Execute()
{
timesMoved=timesMoved+1;
}

// Make this return true when this Command no longer needs to run execute()
bool Autonomous::IsFinished()
{
	if(timesMoved>=25)return true;
	return false;
}

// Called once after isFinished returns true
void Autonomous::End()
{

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Autonomous::Interrupted()
{
	Robot:: driveTrain->Stop();
}
