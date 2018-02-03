#include "PointTurnRight.h"
#include "../../Robot.h"

PointTurnRight::PointTurnRight(double seconds, double power): frc::Command() {
	count = 0;
	m_ticksToExecute = seconds * 50;		// because Execute() is run at 50Hz
	m_power = power;
	Requires (Robot::driveBase.get());
}

// Called just before this Command runs the first time
void PointTurnRight::Initialize() {
	//robot turns right
	count = 0;
	Robot::driveBase->SetLeftPower(-m_power);
	Robot::driveBase->SetRightPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void PointTurnRight::Execute() {
	count = count + 1;
}

// Make this return true when this Command no longer needs to run execute()
bool PointTurnRight::IsFinished() {
	return count >= m_ticksToExecute;
}

// Called once after isFinished returns true
void PointTurnRight::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PointTurnRight::Interrupted() {
	End();
}
