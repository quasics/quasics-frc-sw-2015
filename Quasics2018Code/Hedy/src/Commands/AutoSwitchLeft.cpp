#include <Commands/AutoSwitchLeft.h>
#include "MoveForDistance.h"
#include "PointTurnRight.h"
#include "ShoulderMove.h"
#include "Outtake.h"
AutoSwitchLeft::AutoSwitchLeft() {

	AddSequential(new MoveForDistance(192, .4));
	//14 feet
	AddSequential(new PointTurnRight(4, .4));
	AddSequential(new ShoulderMove(4, .4));
	AddSequential(new Outtake(4, .4));
	// I used 4 and .4 just to start may need to be adjusted
}
