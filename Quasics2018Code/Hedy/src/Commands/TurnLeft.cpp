/*
 * TurnLeft.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: sth101
 */

#include "TurnLeft.h"
#include "Robot.h"

TurnLeft::TurnLeft(double degrees)
: DegreesToTurn(degrees)
{
	// TODO Auto-generated constructor stub
	Requires(Robot::driveBase.get());
	Requires(Robot::navigation.get());
}

void TurnLeft::Initialize() {
	double DegreesStart = Robot::navigation->getAngle();
	Goal = DegreesStart - DegreesToTurn;
	// Start the motors running....
	Robot::driveBase->SetLeftPower(-.25);
	Robot::driveBase->SetRightPower(.25);
}

void TurnLeft::Execute() {
}

bool TurnLeft::IsFinished() {
	return(Robot::navigation->getAngle() < Goal);
}

void TurnLeft::End() {
	Robot::driveBase->Stop();
}

void TurnLeft::Interrupted() {
	Robot::driveBase->Stop();
}


