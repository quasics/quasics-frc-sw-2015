#include "ExperimentalTwo.h"
#include "../Robot.h"

ExperimentalTwo::ExperimentalTwo()
{
	Requires(Robot::driveBase.get());

}

void ExperimentalTwo::Initialize()
{

	Robot::driveBase->SetLeftPower(-.25);
	Robot::driveBase->SetRightPower(.25);

}

void ExperimentalTwo::Execute()
{



}

bool ExperimentalTwo::IsFinished()
{

	// TODO: Add an implementation as needed.  (Mr. Healy added the next line
	// in order to eliminate a compiler warning.)
	return false;

}
void ExperimentalTwo::End()
{
	Robot::driveBase->Stop();


}
void ExperimentalTwo::Interrupted()
{

	Robot::driveBase->Stop();

}
