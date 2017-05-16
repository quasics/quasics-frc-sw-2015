#include "AutoTurnLeft.h"

AutoTurnLeft::AutoTurnLeft(double seconds, double powerLevel) {
	Requires(Robot::driveBase.get());
	m_seconds = seconds;
	power = powerLevel;
	counter = 0;
}

// Called just before this Command runs the first time
void AutoTurnLeft::Initialize() {
	Robot::driveBase->RightEncoderReset();
	counter = 0;
	Robot::driveBase->SetRightPower(power);
}

// Called repeatedly when this Command is scheduled to run
void AutoTurnLeft::Execute() {
	counter+=1;
	if (m_seconds <1.5) {
		power = .2;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutoTurnLeft::IsFinished() {
	return counter >= m_seconds*50;
}

// Called once after isFinished returns true
void AutoTurnLeft::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoTurnLeft::Interrupted() {
	Robot::driveBase->Stop();
}
