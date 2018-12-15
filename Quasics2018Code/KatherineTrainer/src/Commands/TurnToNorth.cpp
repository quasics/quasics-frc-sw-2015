/*
 * TurnToNorth.cpp
 *
 *  Created on: Dec 15, 2018
 *      Author: Developer
 */

#include <Commands/TurnToNorth.h>
#include "../Robot.h"
#include <Subsystems/Navigation.h>

TurnToNorth::TurnToNorth() {
	// TODO Auto-generated constructor stub
Requires(Robot::driveBase.get());
Requires(Robot::navigation.get());

}
void TurnToNorth::Initialize() {
	if (Robot::navigation->getCompassHeadingRelativeToNorth() < 0) {
		Robot::driveBase->SetPowerToMotors(.15, -.15);
	} else {
		Robot::driveBase->SetPowerToMotors(-.15, .15);
	}
}

bool TurnToNorth::IsFinished() {
	return (Robot::navigation->getCompassHeadingRelativeToNorth() > -1)
			&& (Robot::navigation->getCompassHeadingRelativeToNorth() < 1);

}
void TurnToNorth::End() {
	Robot::driveBase->Stop();
}

void TurnToNorth::Interrupted() {
	End();
}


