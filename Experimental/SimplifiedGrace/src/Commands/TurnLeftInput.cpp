#include <Commands/TurnLeftInput.h>
#include "../Robot.h"

TurnLeftInput::TurnLeftInput(double degrees)
: DegreesToTurn(degrees)
{
	// TODO Auto-generated constructor stub
	Requires(Robot::driveBase.get());
	Requires(Robot::navigation.get());
}

void TurnLeftInput::Initialize() {
	double DegreesStart = Robot::navigation->getBearing();
	Goal = DegreesStart - DegreesToTurn;
	if(Goal < -180){
		Goal = Goal + 360;
	}
	// See if we're going to pass over the "south" line, where
	// the bearing flips from -180 to +180.
	PassingSouth = (Goal>0 && DegreesStart<0);
	// Start the motors running....
	Robot::driveBase->SetLeftPower(-.25);
	Robot::driveBase->SetRightPower(.25);
}

void TurnLeftInput::Execute() {
}

bool TurnLeftInput::IsFinished() {
		double DegreesTurning = Robot::navigation->getBearing();
		bool NotPassedYet = DegreesTurning > -180;
		if(PassingSouth && NotPassedYet){
			return false;
		}
		else{
			return(Goal<DegreesTurning);
		}
	}

void TurnLeftInput::End() {
	Robot::driveBase->Stop();
}

void TurnLeftInput::Interrupted() {
	Robot::driveBase->Stop();
}

