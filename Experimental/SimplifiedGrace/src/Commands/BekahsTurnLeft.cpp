/*
 * BekahsTurnLeft.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: rad100
 */

#include "BekahsTurnLeft.h"
#include "../Robot.h"

BekahsTurnLeft::BekahsTurnLeft() {
	// TODO Auto-generated constructor stub
	Requires(Robot::driveBase.get());
	Requires(Robot::navigation.get());
}

void BekahsTurnLeft::Initialize() {
	Robot::navigation->resetBearing();
	Robot::driveBase->SetLeftPower(-.25);
	Robot::driveBase->SetRightPower(.25);
}

void BekahsTurnLeft::Execute() {
	// Nothing to do for this command....
}

bool BekahsTurnLeft::IsFinished() {
	/*
	 * if (someKindOfTest) {
	 * 		// Do something
	 * } else {
	 * 		// Do something else
	 * }
	 */
	if (Robot::navigation->getBearing()<=-90) {
		return true;
	} else {
		return false;

	}
}

void BekahsTurnLeft::End() {
	Robot::driveBase->Stop();
}

void BekahsTurnLeft::Interrupted() {
	Robot::driveBase->Stop();
}
