#include <Commands/AutoSwitchLeft.h>
#include "MoveForDistance.h"
#include "PointTurnLeft.h"
#include "ShoulderMove.h"
#include "Outtake.h"
AutoSwitch::AutoSwitch() {

	AddSequential(new MoveForDistance(192, .4));
	//14 feet
	AddSequential(new PointTurnLeft(4, .4));
	AddSequential(new ShoulderMove(4, .4));
	AddSequential(new Outtake(4, .4));
	// I used 4 and .4 just to start may need to be adjusted
}
