/*
 * ExecuteTurn.cpp
 *
 *  Created on: Nov 4, 2017
 *      Author: Developer
 */

#include "ExecuteTurn.h"
#include "../Robot.h"

#include <iostream>
#include <iomanip>

//
// Debugging control flags
//

// If defined, disable motor engagement (so that we will just report angles if NOISY is defined).
#undef DONT_RUN_MOTORS

// If defined, report angular position data while command is executing.
#undef NOISY

ExecuteTurn::ExecuteTurn(float degrees, double powerLevel)
: degrees(degrees), powerLevel(powerLevel), targetHeading(0)
{
	Requires (Robot::driveBase.get());
	Requires (Robot::navigation.get());
}

void ExecuteTurn::Initialize() {
	double leftPower = powerLevel, rightPower = powerLevel;
	if (degrees < 0) {
		// Need to turn to the left
		leftPower *= -1;
	} else {
		// Need to turn to the right
		rightPower *= -1;
	}

	float startingPosition = Robot::navigation->getAngle();
	targetHeading = (startingPosition + degrees);

#ifdef NOISY
	std::cout << "Starting turn by " << degrees << " degrees" << std::endl;
	std::cout << "Target angular bearing is: " << targetHeading << std::endl;
	std::cout << "Compass heading: " << std::setw(7) << Robot::navigation->getCompassHeading()
			  << " Angular bearing: " << std::setw(7) << startingPosition << std::endl;
#endif	// NOISY

#ifdef DONT_RUN_MOTORS
	Robot::driveBase->Stop();
#else
	Robot::driveBase->SetPower(leftPower, rightPower);
#endif	// DONT_RUN_MOTORS
}

void ExecuteTurn::Execute() {
#ifdef NOISY
	std::cout << "Compass heading: " << std::setw(7) << Robot::navigation->getCompassHeading()
			  << " Angular bearing: " << std::setw(7) << Robot::navigation->getAngle() << std::endl;
#endif	// NOISY
}

bool ExecuteTurn::IsFinished() {
	if (degrees < 0) {
		return (Robot::navigation->getAngle() <= targetHeading);
	} else {
		return (Robot::navigation->getAngle() >= targetHeading);
	}
}

void ExecuteTurn::End() {
	std::cout << "Final bearing is " << Robot::navigation->getAngle() << std::endl;
	Robot::driveBase->Stop();
}

void ExecuteTurn::Interrupted() {
	End();
}
