#include "DancingRobot.h"
#include "MoveForward.h"
#include "TurnLeftFortyFive.h"
#include "TurnRightFortyFive.h"

DancingRobot::DancingRobot() {
	AddSequential(new TurnLeftFortyFive());
	AddSequential(new TurnRightFortyFive());
	AddSequential(new TurnRightFortyFive());
	AddSequential(new TurnLeftFortyFive());
	AddSequential(new TurnRightFortyFive());
	AddSequential(new TurnLeftFortyFive());
	AddSequential(new TurnLeftFortyFive());
	AddSequential(new TurnRightFortyFive());
	AddSequential(new MoveForward(.1,.5));
	AddSequential(new MoveForward(-.1,.5));
	AddSequential(new TurnRightFortyFive());
	AddSequential(new TurnRightFortyFive());
	AddSequential(new TurnRightFortyFive());
	AddSequential(new TurnRightFortyFive());
	AddSequential(new TurnRightFortyFive());
	AddSequential(new TurnRightFortyFive());
	AddSequential(new TurnRightFortyFive());
	AddSequential(new TurnRightFortyFive());
}
