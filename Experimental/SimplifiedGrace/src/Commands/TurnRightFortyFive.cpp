/*
 * TurnRightFortyFive.cpp
 *
 *  Created on: Nov 10, 2017
 *      Author: sth101
 */

#include "TurnRightFortyFive.h"
#include "../Robot.h"

TurnRightFortyFive::TurnRightFortyFive() {
	// TODO Auto-generated constructor stub
	Requires(Robot::driveBase.get());
	Requires(Robot::navigation.get());
}

TurnRightFortyFive::~TurnRightFortyFive() {
	// TODO Auto-generated destructor stub
}


// Called just before this Command runs the first time
void TurnRightFortyFive::Initialize() {
	Robot::driveBase->SetLeftPower(.10);
	Robot::driveBase->SetRightPower(-.10);
	Robot::navigation->resetBearing();
}

// Called repeatedly when this Command is scheduled to run
void TurnRightFortyFive::Execute() {
	// There's nothing to do during execution: the Initialize() code starts
	// the wheels turning.
}

// Make this return true when this Command no longer needs to run execute()
bool TurnRightFortyFive::IsFinished() {
	return (Robot::navigation->getBearing() > 45);
}

// Called once after isFinished returns true
void TurnRightFortyFive::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TurnRightFortyFive::Interrupted() {
	Robot::driveBase->Stop();
}
