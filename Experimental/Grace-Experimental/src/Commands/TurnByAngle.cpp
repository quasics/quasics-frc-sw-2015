/*
 * TurnByAngle.cpp
 *
 *  Created on: Apr 14, 2018
 *      Author: healym
 */

#include "TurnByAngle.h"
#include "../Robot.h"

constexpr double LEFT_POWER = .25;
constexpr double RIGHT_POWER = -.25;

namespace {
	// Converts an angle (in degrees) to a value in the range [0..360).
	double normalizeAngle(double angleDegrees) {
		double result = 0;
		if (angleDegrees >= 0 && angleDegrees < 360) {
			result = angleDegrees;
		} else if (angleDegrees < 0) {
			// Angle is negative (and for now, we'll only turn to the left), so
			// convert it to a positive value.
			result = angleDegrees - (((int(angleDegrees) / 360) - 1) * 360);
		} else {
			// Angle is too big: reduce it
			result = angleDegrees - ((int(angleDegrees) / 360) * 360);
		}
		return result;
	}
}

TurnByAngle::TurnByAngle(double angleDegrees)
: angleDegrees(normalizeAngle(angleDegrees))
{
	Requires (Robot::driveBase.get());
	Requires (Robot::navAlt.get());
}

// Called just before this Command runs the first time
void TurnByAngle::Initialize() {
	Robot::navAlt->resetGyro();

	// Note: This is going to produce pretty jerky motion, especially at
	// anything near full power.  A better approach might be to start out at
	// low power, ramping up to speed (in an overload of the execute()
	// function), and ramping down as we get close to the end of the allotted
	// time.
	Robot::driveBase->SetPower(LEFT_POWER, RIGHT_POWER);
}

// Make this return true when this Command no longer needs to run execute()
bool TurnByAngle::IsFinished() {
	return Robot::navAlt->getGyroValue() >= angleDegrees;
}

// Called once after isFinished returns true
void TurnByAngle::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TurnByAngle::Interrupted() {
	End();
}
