#include "MoveForward.h"

MoveForward::MoveForward()
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
	Requires (Robot::driveTrain.get());
}

// Called just before this Command runs the first time
void MoveForward::Initialize()
{
	Robot::driveTrain->SetLeftPower(.5);
	Robot::driveTrain->SetRightPower(.5);
	counter = 0;

}

// Called repeatedly when this Command is scheduled to run
void MoveForward::Execute()
{
counter += 1;
}

// Make this return true when this Command no longer needs to run execute()
bool MoveForward::IsFinished()
{
	return false;
}

// Called once after isFinished returns true
void MoveForward::End()
{
Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForward::Interrupted()
{
	Robot::driveTrain->Stop();

}
