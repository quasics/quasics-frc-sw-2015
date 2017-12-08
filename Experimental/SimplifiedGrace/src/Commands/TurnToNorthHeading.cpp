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
	if (currentHeading > 10 ){
	//	std::cout << "Heading is positive: turning left" << std::endl;
		Robot::driveBase->SetLeftPower(-.25);
		Robot::driveBase->SetRightPower(.25);
		wasTurningToLeft = true;
	} else if (currentHeading < -10){
	//	std::cout << "Heading is negative: turning right" << std::endl;
		Robot::driveBase->SetLeftPower(.25);
		Robot::driveBase->SetRightPower(-.25);
		wasTurningToLeft = false;
	} else if (currentHeading < 5 && currentHeading > -5) {
		Robot::driveBase->SetLeftPower(0);
		Robot::driveBase->SetRightPower(0);
	}

}

void TurnToNorthHeading::Execute(){
	// Nothing to do: motors were started in Initialize().
}

bool TurnToNorthHeading::IsFinished() {
	double currentHeading = Robot::navigation->getCompassHeading();
	if (wasTurningToLeft) {
	//	std::cout << "(L) Current heading: " << currentHeading << std::endl;
		if (currentHeading > -5 && currentHeading < 5) {
			// Either at North, or (hopefully slightly) overshot
			return true;
		}
	} else if (!wasTurningToLeft) {
	//	std::cout << "(R) Current heading: " << currentHeading << std::endl;
		if (currentHeading >= 0) {
			// Either at North, or (hopefully slightly) overshot
			return true;
		}
	//	std::cout << "Heading is still negative: continuing turn" << std::endl;
	}
	return false;
}

void TurnToNorthHeading::End() {
	Robot::driveBase->Stop();
}

void TurnToNorthHeading::Interrupted() {
	End();
}
