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
	std::cout << "Initial heading: " << currentHeading << std::endl;
	if (currentHeading > 0 ){
		std::cout << "Heading is positive: turning left" << std::endl;
		Robot::driveBase->SetLeftPower(-.25);
		Robot::driveBase->SetRightPower(.25);
		wasTurningToLeft = true;
	} else if (currentHeading < 0){
		std::cout << "Heading is negative: turning right" << std::endl;
		Robot::driveBase->SetLeftPower(.25);
		Robot::driveBase->SetRightPower(-.25);
		wasTurningToLeft = false;
	}

}

void TurnToNorthHeading::Execute(){
	// Nothing to do: motors were started in Initialize().
}

bool TurnToNorthHeading::IsFinished() {
	double currentHeading = Robot::navigation->getCompassHeading();
	std::cout << "Current heading: " << currentHeading << std::endl;
	if (wasTurningToLeft) {
		if (currentHeading <= 0) {
			// Either at North, or (hopefully slightly) overshot
			return true;
		}
		std::cout << "Heading is still positive: continuing turn" << std::endl;
	} else if (!wasTurningToLeft) {
		if (currentHeading >= 0) {
			// Either at North, or (hopefully slightly) overshot
			return true;
		}
		std::cout << "Heading is still negative: continuing turn" << std::endl;
	}
	return false;
}

void TurnToNorthHeading::End() {
	Robot::driveBase->Stop();
}

void TurnToNorthHeading::Interrupted() {
	End();
}
