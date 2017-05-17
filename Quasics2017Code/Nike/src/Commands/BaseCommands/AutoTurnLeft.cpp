#include "AutoTurnLeft.h"

AutoTurnLeft::AutoTurnLeft(double seconds, double powerLevel)
: m_seconds(seconds), power(powerLevel) {
	Requires(Robot::driveBase.get());
	counter = 0;
}

// Called just before this Command runs the first time
void AutoTurnLeft::Initialize() {
	Robot::driveBase->RightEncoderReset();
	counter = 0;
	Robot::driveBase->SetRightPower(0);		// Power gets ramped up during "Execute()"
}

// Called repeatedly when this Command is scheduled to run
void AutoTurnLeft::Execute() {
	counter+=1;

	/*
	 * Note: the following will only be done when the # of seconds is > .2 seconds, or 10 cycles
	 */
	int maxCycles = int(m_seconds*50);
	const int cyclesForChange = 5;
	if (counter <= cyclesForChange) {
		// Speed up
		double percentPowerApplied = counter / double(cyclesForChange);
		Robot::driveBase->SetRightPower(power * percentPowerApplied);
	} else if (counter > (maxCycles - cyclesForChange) ) {
		// Slow down
		double percentPowerApplied = (150 - cyclesForChange) / double(cyclesForChange);
		Robot::driveBase->SetRightPower(power * percentPowerApplied);
	} else {
		// Running at full power....
		Robot::driveBase->SetRightPower(power);
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
