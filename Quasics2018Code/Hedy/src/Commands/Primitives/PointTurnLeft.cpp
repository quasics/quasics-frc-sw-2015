#include "PointTurnLeft.h"
#include "../../Robot.h"

PointTurnLeft::PointTurnLeft(double seconds, double power): frc::Command() {
	count = 0;
	m_ticksToExecute = seconds;		// because Execute() is run at 50Hz
	m_power = power;
	Requires (Robot::driveBase.get());
}

// Called just before this Command runs the first time
void PointTurnLeft::Initialize() {
	//robot turns left
	count = 0;
	Robot::driveBase->SetLeftPower(m_power);
	Robot::driveBase->SetRightPower(-m_power);
}

// Called repeatedly when this Command is scheduled to run
void PointTurnLeft::Execute() {
	count = count + 1;
}

// Make this return true when this Command no longer needs to run execute()
bool PointTurnLeft::IsFinished() {
	return count >= m_ticksToExecute;
}

// Called once after isFinished returns true
void PointTurnLeft::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PointTurnLeft::Interrupted() {
	End();
}
