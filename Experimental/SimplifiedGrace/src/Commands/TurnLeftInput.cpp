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
	double DegreesStart = Robot::navigation->getAngle();
	Goal = DegreesStart - DegreesToTurn;
	// Start the motors running....
	Robot::driveBase->SetLeftPower(-.25);
	Robot::driveBase->SetRightPower(.25);
}

void TurnLeftInput::Execute() {
}

bool TurnLeftInput::IsFinished() {
	return(Robot::navigation->getAngle() < Goal);
}

void TurnLeftInput::End() {
	Robot::driveBase->Stop();
}

void TurnLeftInput::Interrupted() {
	Robot::driveBase->Stop();
}

