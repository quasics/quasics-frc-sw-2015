#include "TurnLeftInput.h"
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
	std::cout << "Goal orientation is: " << Goal << std::endl;
	// See if we're going to pass over the "south" line, where
	// the bearing flips from -180 to +180.
	PassingSouth = (Goal>0 && DegreesStart<0);
	std::cout << "PassingSouth == " << int(PassingSouth) << std::endl;
	// Start the motors running....
	Robot::driveBase->SetLeftPower(-.25);
	Robot::driveBase->SetRightPower(.25);
}

void TurnLeftInput::Execute() {
}

bool TurnLeftInput::IsFinished() {
		double DegreesTurning = Robot::navigation->getBearing();
		// Note: this isn't complete, and will just cover the case where
		// we're west of North.  If we start @ 10 degrees to east, for
		// instance, we won't cover that case here.
		bool NotPassedYet = DegreesTurning < 0 && DegreesTurning > -180;
		std::cout << "Current heading " << DegreesTurning << std::endl;
		std::cout << "NotPassetYet " << int(NotPassedYet) << std::endl;

		if(PassingSouth && NotPassedYet){
			return false;
		}
		else{
			return(Goal>DegreesTurning);
		}
	}

void TurnLeftInput::End() {
	Robot::driveBase->Stop();
}

void TurnLeftInput::Interrupted() {
	Robot::driveBase->Stop();
}

