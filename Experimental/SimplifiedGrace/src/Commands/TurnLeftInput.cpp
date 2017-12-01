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
	Robot::driveBase->SetLeftPower(-.25);
	Robot::driveBase->SetRightPower(.25);
}

void TurnLeftInput::Execute() {
}

bool TurnLeftInput::IsFinished() {
	double DegreesNow = Robot::navigation->getBearing();
	return (Robot::navigation->getBearing() > DegreesNow - DegreesToTurn);
}

void TurnLeftInput::End() {
	Robot::driveBase->Stop();
}

void TurnLeftInput::Interrupted() {
	Robot::driveBase->Stop();
}

