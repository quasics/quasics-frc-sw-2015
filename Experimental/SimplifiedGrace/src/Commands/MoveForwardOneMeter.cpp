#include "MoveForwardOneMeter.h"
#include "../Robot.h"

MoveForwardOneMeter::MoveForwardOneMeter()
{
	// TODO Auto-generated constructor stub
	Requires(Robot::driveBase.get());
	Requires(Robot::navigation.get());
}

void MoveForwardOneMeter::Initialize() {
	// Start the motors running....
	Robot::driveBase->SetLeftPower(.25);
	Robot::driveBase->SetRightPower(.25);
	Robot::navigation->resetForwardMotion();
}

void MoveForwardOneMeter::Execute() {
}

bool MoveForwardOneMeter::IsFinished() {
	int Goal = 1;
	return Robot::navigation->getForwardMotion()<Goal;
}

void MoveForwardOneMeter::End() {
	Robot::driveBase->Stop();
}

void MoveForwardOneMeter::Interrupted() {
	Robot::driveBase->Stop();
}
