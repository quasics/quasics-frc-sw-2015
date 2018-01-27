#include "AutoStartMid.h"
#include "AutoSwitchLeft.h"
#include "AutoSwitchRight.h"
#include "PointTurnLeft.h"
#include "PointTurnRight.h"
#include "AutoFMS.h"
#include "MoveForDistance.h"
AutoStartMid::AutoStartMid() {
	AddSequential(new AutoFMS());
	if(gameData[0] == 'L'){
		//left
		AddSequential(new AutoSwitchLeft());
	}
	else {
		// move away from wall to turn
		AddSequential(new MoveForDistance(12, .4));
		//turn right move forward turn left
		AddSequential(new PointTurnRight(4, .4));
		AddSequential(new MoveForDistance(132, .4));
		//11 feet
		AddSequential(new PointTurnLeft(4, .4));
		//right
		AddSequential(new AutoSwitchRight());
	}
}
