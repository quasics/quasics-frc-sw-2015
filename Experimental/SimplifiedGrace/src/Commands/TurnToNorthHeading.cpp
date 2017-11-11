/*
 * TurnToCompassHeading.cpp
 *
 *  Created on: Nov 10, 2017
 *      Author: sxm100
 */

#include "TurnToNorthHeading.h"

#include "../Robot.h"

TurnToNorthHeading::TurnToNorthHeading() {
	// TODO Auto-generated constructor stub
	Requires(Robot::driveBase.get());
	Requires(Robot::navigation.get());
}

void TurnToNorthHeading::Initialize() {
	double currentHeading = Robot::navigation->getCompassHeading();
	Robot::driveBase->SetLeftPower(0);
	Robot::driveBase->SetRightPower(0);
	if (currentHeading > 0 ){
		Robot::driveBase->SetLeftPower(-.25);
		Robot::driveBase->SetRightPower(.25);
		wasTurningToLeft = true;
	} else if (currentHeading < 0){
		Robot::driveBase->SetLeftPower(.25);
		Robot::driveBase->SetRightPower(-.25);
		wasTurningToLeft = false;
	}

}

void TurnToNorthHeading::Execute(){
	// Nothing to do: motors were started in Initialize().
}

bool TurnToNorthHeading::IsFinished() {
	if (wasTurningToLeft) {
		if (Robot::navigation->getCompassHeading() <= 0) {
			// Either at North, or (hopefully slightly) overshot
			return true;
		}
	} else if (!wasTurningToLeft) {
		if (Robot::navigation->getCompassHeading() >= 0) {
			// Either at North, or (hopefully slightly) overshot
			return true;
		}
	}
	return false;
}

void TurnToNorthHeading::End() {
	Robot::driveBase->Stop();
}

void TurnToNorthHeading::Interrupted() {
	End();
}
