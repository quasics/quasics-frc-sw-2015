#include "MoveAheadTime.h"
#include "../Robot.h"
MoveAheadTime::MoveAheadTime(double seconds, double power): Command ()
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
	m_seconds = seconds;
	m_power = power;
	Requires (Robot::driveTrain.get());

}

// Called just before this Command runs the first time
void MoveAheadTime::Initialize()
{
	counter = 0;
	Robot::driveTrain->SetLeftPower(m_power);
	Robot::driveTrain->SetRightPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void MoveAheadTime::Execute()
{
	counter+=1;
}

// Make this return true when this Command no longer needs to run execute()
bool MoveAheadTime::IsFinished()
{
	return counter >= m_seconds*50;
}

// Called once after isFinished returns true
void MoveAheadTime::End()
{
	Robot::driveTrain->Stop();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveAheadTime::Interrupted()
{
	Robot::driveTrain->Stop();
}
