#include "Arman.h"

#include "PointTurnForAngle.h"
#include "MoveForTime.h"

Arman::Arman() {
	AddSequential(new PointTurnForAngle(45,.3));
	AddSequential(new MoveForTime(3,.5));
	AddSequential(new PointTurnForAngle(90,.3));
	AddSequential(new MoveForTime(3,.5));
	AddSequential(new PointTurnForAngle(-45,.3));
	AddSequential(new MoveForTime(1,.5));
	AddSequential(new PointTurnForAngle(-90,.3));
	AddSequential(new MoveForTime(3,.5));
	AddSequential(new PointTurnForAngle(90,.3));
	AddSequential(new MoveForTime(1.5,.5));
	AddSequential(new MoveForTime(-1.5,.5));
	AddSequential(new PointTurnForAngle(90,.3));
	AddSequential(new MoveForTime(1.5,.5));
	AddSequential(new PointTurnForAngle(-90,.3));
	AddSequential(new MoveForTime(1.5,.5));
	AddSequential(new MoveForTime(-1.5,.5));
	AddSequential(new PointTurnForAngle(-90,.3));
	AddSequential(new MoveForTime(1.5,.5));
}
