#include "SwingTurnRight.h"
#include "../../Robot.h"

SwingTurnRight::SwingTurnRight(double seconds, double power): frc::Command() {
	count = 0;
	m_seconds = seconds;
	m_power = power;
	Requires (Robot::driveBase.get());
}

// Called just before this Command runs the first time
void SwingTurnRight::Initialize() {
	//robot turns right without using left motors
	Robot::driveBase->SetLeftPower(0);
	Robot::driveBase->SetRightPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void SwingTurnRight::Execute() {
	while(count <= m_seconds){
		count = count + 1;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool SwingTurnRight::IsFinished() {
	return count >= m_seconds;
}

// Called once after isFinished returns true
void SwingTurnRight::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SwingTurnRight::Interrupted() {
	End();
}
