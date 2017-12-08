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



}
void ExperimentalTwo::End()
{
	Robot::driveBase->Stop();


}
void ExperimentalTwo::Interrupted()
{

	Robot::driveBase->Stop();

}
