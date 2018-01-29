/*
 * TurnToCompassHeading.cpp
 *
 *  Created on: Nov 10, 2017
 *      Author: sxm100
 */

#include "TurnToNorthHeading.h"

#include "../Robot.h"
#include <iostream>

// CODE_REVIEW(mjh): You're not initializing "wasTurningToLeft" in here.  As a
// result, you're going to get a "garbage value" for this field, and it's likely
// to evaluate as "true".
TurnToNorthHeading::TurnToNorthHeading() {
	Requires(Robot::driveBase.get());
	Requires(Robot::navigation.get());
}

void TurnToNorthHeading::Initialize() {
	double currentHeading = Robot::navigation->getCompassHeading();
	Robot::driveBase->SetLeftPower(0);
	Robot::driveBase->SetRightPower(0);
	std::cout << "Trying to turn to North (heading 0)" << std::endl;
	if (currentHeading > 10 || currentHeading < 350){
		std::cout << "Heading is positive: turning left" << std::endl;
		Robot::driveBase->SetLeftPower(-.25);
		Robot::driveBase->SetRightPower(.25);
		wasTurningToLeft = true;
	} else {
		Robot::driveBase->Stop();
	}

}

void TurnToNorthHeading::Execute(){
	// Nothing to do: motors were started in Initialize().
}

bool TurnToNorthHeading::IsFinished() {
	double currentHeading = Robot::navigation->getCompassHeading();
	if (wasTurningToLeft) {
		std::cout << "(L) Current heading: " << currentHeading << std::endl;
		if (currentHeading > 350 || currentHeading < 10) {
			// Either at North, or (hopefully slightly) overshot
			std::cout << "Stopping!" << std::endl;
			return true;
		}
	} else if (!wasTurningToLeft) {
		std::cout << "(R) Current heading: " << currentHeading << std::endl;
		if (currentHeading > 350 || currentHeading < 10) {
			// Either at North, or (hopefully slightly) overshot
			std::cout << "Stopping!" << std::endl;
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
