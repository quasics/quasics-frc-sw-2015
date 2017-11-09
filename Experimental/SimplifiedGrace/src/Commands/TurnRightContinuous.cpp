#include "TurnRightContinuous.h"
#include "../Robot.h"

TurnRightContinuous::TurnRightContinuous()
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::driveBase.get());
}

void TurnRightContinuous::Initialize() {
	Robot::driveBase->SetLeftPower(.25);
	Robot::driveBase->SetRightPower(-.25);
}

void TurnRightContinuous::Execute() {
	// Nothing to do for this command....
}

bool TurnRightContinuous::IsFinished() {
	return false;
}

void TurnRightContinuous::End() {
	Robot::driveBase->Stop();
}

void TurnRightContinuous::Interrupted() {
	Robot::driveBase->Stop();
}
