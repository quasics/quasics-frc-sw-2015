#include "TurnRightInput.h"
#include "../Robot.h"

TurnRightInput::TurnRightInput(double degrees)
: DegreesToTurn(degrees), Goal(0)
{
	// TODO Auto-generated constructor stub
	Requires(Robot::driveBase.get());
	Requires(Robot::navigation.get());
}

void TurnRightInput::Initialize() {
	double DegreesStart = Robot::navigation->getAngle();
	Goal = DegreesStart + DegreesToTurn;
	// Start the motors running....
	Robot::driveBase->SetLeftPower(.25);
	Robot::driveBase->SetRightPower(-.25);
}

void TurnRightInput::Execute() {
}

bool TurnRightInput::IsFinished() {
	return(Robot::navigation->getAngle() > Goal);
}

void TurnRightInput::End() {
	Robot::driveBase->Stop();
}

void TurnRightInput::Interrupted() {
	Robot::driveBase->Stop();
}
