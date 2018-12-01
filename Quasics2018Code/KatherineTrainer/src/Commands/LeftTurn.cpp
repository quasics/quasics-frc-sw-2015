/*
 * LeftTurn.cpp
 *
 *  Created on: Dec 1, 2018
 *      Author: Developer
 */

#include <Commands/LeftTurn.h>
#include <Robot.h>

LeftTurn::LeftTurn(double duration, double percentPower)
	: TimedCommand(duration), percentPower_(percentPower)
{
	// TODO Auto-generated constructor stub
	Requires(Robot::driveBase.get());
}

void LeftTurn::Initialize(){
	Robot::driveBase->SetPowerToMotors(percentPower_, -percentPower_);
}
void LeftTurn::End(){
	Robot::driveBase->Stop();
}
void LeftTurn::Interrupted(){
	End();
}
