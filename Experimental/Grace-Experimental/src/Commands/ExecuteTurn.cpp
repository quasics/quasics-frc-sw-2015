/*
 * ExecuteTurn.cpp
 *
 *  Created on: Nov 4, 2017
 *      Author: Developer
 */

#include "ExecuteTurn.h"
#include "../Robot.h"

ExecuteTurn::ExecuteTurn(float degrees, double powerLevel)
: degrees(degrees), powerLevel(powerLevel)
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

	std::cout << "Starting turn by " << degrees << " degrees" << std::endl;
	float startingPosition = Robot::navigation->getAngle();
	targetHeading = (startingPosition + degrees);

	std::cout << "Current bearing is: " << startingPosition << std::endl;
	std::cout << "Target bearing is: " << targetHeading << std::endl;
	Robot::driveBase->SetLeftPower(leftPower);
	Robot::driveBase->SetRightPower(rightPower);
}

void ExecuteTurn::Execute() {
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
