/*
 * TimedMove.cpp
 *
 *  Created on: Dec 1, 2018
 *      Author: healym
 */

#include "TimedMove.h"
#include "../Robot.h"

TimedMove::TimedMove(double duration, double percentPower)
	: TimedCommand(duration), leftPercentPower_(percentPower), rightPercentPower_(percentPower)
{
	Requires(Robot::driveBase.get());
}

TimedMove::TimedMove(double duration, double leftPercentPower, double rightPercentPower)
	: TimedCommand(duration), leftPercentPower_(leftPercentPower), rightPercentPower_(rightPercentPower)
{
	Requires(Robot::driveBase.get());
}

void TimedMove::Initialize() {
	Robot::driveBase->SetPowerToMotors(leftPercentPower_, rightPercentPower_);
}

void TimedMove::End() {
	Robot::driveBase->Stop();
}

void TimedMove::Interrupted() {
	End();
}
