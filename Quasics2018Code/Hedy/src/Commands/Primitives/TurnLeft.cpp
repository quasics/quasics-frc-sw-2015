/*
 * TurnLeft.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: sth101
 */

#include "TurnLeft.h"
#include "Robot.h"

TurnLeft::TurnLeft(double degrees)
: degreesToTurn(degrees), goal(0)
{
	Requires(Robot::driveBase.get());
	Requires(Robot::navigation.get());
}

void TurnLeft::Initialize() {
	goal = Robot::navigation->getAngle() - degreesToTurn;
	// Start the motors running....
	Robot::driveBase->SetPowerToMotors(-.25, .25);
}

bool TurnLeft::IsFinished() {
	return(Robot::navigation->getAngle() < goal);
}

void TurnLeft::End() {
	Robot::driveBase->Stop();
}

void TurnLeft::Interrupted() {
	End();
}


