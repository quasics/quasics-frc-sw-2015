#include "TurnLeft90.h"
#include <Commands/Rotate.h>
#include "../Robot.h"

TurnLeft90::TurnLeft90(double seconds, double power)
{
	m_seconds = seconds;
	m_power = power;
	counter = 0;
	Requires (Robot::driveTrain.get());
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
}

// Called just before this Command runs the first time
void TurnLeft90::Initialize()
{
	counter = 0;
	Robot::driveTrain->SetLeftPower(-m_power);
	Robot::driveTrain->SetRightPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void TurnLeft90::Execute()
{
	counter+=1;
}

// Make this return true when this Command no longer needs to run execute()
bool TurnLeft90::IsFinished()
{
	return counter >= m_seconds*50;
	Robot::driveTrain->Stop();
}

// Called once after isFinished returns true
void TurnLeft90::End()
{
	Robot::driveTrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TurnLeft90::Interrupted()
{
	Robot::driveTrain->Stop();
}
