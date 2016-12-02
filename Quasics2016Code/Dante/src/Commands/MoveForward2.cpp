#include "MoveForward2.h"
#include "../Robot.h"

MoveForward2::MoveForward2(double seconds, double power): Command()
{
	m_seconds = seconds;
	m_power = power;
	counter = 0;
	Requires (Robot::driveTrain.get());

	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
}

// Called just before this Command runs the first time
void MoveForward2::Initialize()
{
	counter = 0;
	Robot::driveTrain->SetLeftPower(m_power);
	Robot::driveTrain->SetRightPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void MoveForward2::Execute()
{
	counter+=1;
}

// Make this return true when this Command no longer needs to run execute()
bool MoveForward2::IsFinished()
{
	return counter >= m_seconds*50;
	Robot::driveTrain->Stop();
	return false;
}

// Called once after isFinished returns true
void MoveForward2::End()
{
	Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForward2::Interrupted()
{
	Robot::driveTrain->Stop();
}
