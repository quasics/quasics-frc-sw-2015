#include "MoveAhead3.h"
#include "../Robot.h"

MoveAhead3::MoveAhead3(double seconds, double power): Command()
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
	m_seconds = seconds;
	m_power = power;
	counter = 0;
	Requires (Robot::driveTrain.get());
}

// Called just before this Command runs the first time
void MoveAhead3::Initialize()
{
	counter = 0;
	Robot::driveTrain->SetLeftPower(m_power);
	Robot::driveTrain->SetRightPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void MoveAhead3::Execute()
{
	counter+=1;
}

// Make this return true when this Command no longer needs to run execute()
bool MoveAhead3::IsFinished()
{
	return counter >= m_seconds*50;
	Robot::driveTrain->Stop();
	return false;
}

// Called once after isFinished returns true
void MoveAhead3::End()
{
	Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveAhead3::Interrupted()
{
	Robot::driveTrain->Stop();
}
