#include "MoveForwardTime.h"
#include "../Robot.h"

MoveForwardTime::MoveForwardTime(double seconds, double power): Command ()
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
	m_seconds = seconds;
	m_power = power;
	Requires (Robot::driveTrain.get());

}

// Called just before this Command runs the first time
void MoveForwardTime::Initialize()
{
	counter = 0;
	Robot::driveTrain->SetLeftPower(m_power);
	Robot::driveTrain->SetRightPower(m_power);
}


// Called repeatedly when this Command is scheduled to run
void MoveForwardTime::Execute()
{
	counter+=1;
}

// Make this return true when this Command no longer needs to run execute()
bool MoveForwardTime::IsFinished()
{
	return counter >= m_seconds*50;
}

// Called once after isFinished returns true
void MoveForwardTime::End()
{
	Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForwardTime::Interrupted()
{
	Robot::driveTrain->Stop();
}
