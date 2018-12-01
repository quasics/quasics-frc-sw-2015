/*
 * TimedMove.cpp
 *
 *  Created on: Dec 1, 2018
 *      Author: healym
 */

#include "TimedMove.h"
#include "../Robot.h"

TimedMove::TimedMove(double duration, double percentPower)
	: TimedCommand(duration), percentPower_(percentPower)
{
	Requires(Robot::driveBase.get());
}

void TimedMove::Initialize() {
	Robot::driveBase->SetPowerToMotors(percentPower_, percentPower_);
}

void TimedMove::End() {
	Robot::driveBase->Stop();
}

void TimedMove::Interrupted() {
	End();
}
