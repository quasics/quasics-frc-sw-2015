#include "TurnRightInput.h"
#include "DancingRobot.h"
#include "TurnLeftInput.h"
#include "MoveForward.h"

DancingRobot::DancingRobot() {
	AddSequential(new TurnRightInput(20));
	AddSequential(new TurnLeftInput(40));
	AddSequential(new TurnRightInput(20));
	AddSequential(new TurnLeftInput(40));
	AddSequential(new TurnRightInput(80));
	AddSequential(new TurnLeftInput(40));
	AddSequential(new TurnRightInput(60));
	AddSequential(new TurnLeftInput(120));
	AddSequential(new TurnRightInput(60));
	AddSequential(new TurnLeftInput(80));
	AddSequential(new TurnRightInput(160));
	AddSequential(new TurnLeftInput(80));
	AddSequential(new TurnRightInput(100));
	AddSequential(new TurnLeftInput(200));
	AddSequential(new TurnRightInput(100));
	AddSequential(new TurnLeftInput(120));
	AddSequential(new TurnRightInput(240));
	AddSequential(new TurnLeftInput(120));
	AddSequential(new TurnRightInput(140));
	AddSequential(new TurnLeftInput(280));
	AddSequential(new TurnRightInput(140));
	AddSequential(new TurnLeftInput(160));
	AddSequential(new TurnRightInput(320));
	AddSequential(new TurnLeftInput(160));
	AddSequential(new TurnRightInput(180));
	AddSequential(new TurnLeftInput(360));
	AddSequential(new TurnRightInput(180));
	AddSequential(new TurnLeftInput(200));
	AddSequential(new TurnRightInput(400));
	AddSequential(new TurnLeftInput(200));
	AddSequential(new MoveForward(.25,2));
	AddSequential(new TurnLeftInput(720));
	AddSequential(new TurnRightInput(720));
	AddSequential(new TurnLeftInput(200));
	AddSequential(new MoveForward(-.25,2));
	AddSequential(new TurnRightInput(720));
	AddSequential(new TurnLeftInput(720));
	AddSequential(new MoveForward(.25,1));
	AddSequential(new MoveForward(-.25,1));
	AddSequential(new TurnRightInput(180));
	AddSequential(new TurnLeftInput(360));
	AddSequential(new TurnRightInput(180));
	AddSequential(new MoveForward(-.25,1));
	AddSequential(new MoveForward(.25,1));
	AddSequential(new TurnLeftInput(180));
	AddSequential(new TurnRightInput(360));
	AddSequential(new TurnLeftInput(180));
	AddSequential(new TurnRightInput(1080));
}
