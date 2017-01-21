#include "MoveInSquare.h"
#include "PointTurnForAngle.h"
#include "MoveForTime.h"

MoveInSquare::MoveInSquare() {
	AddSequential(new MoveForTime(5,.5));
	AddSequential(new PointTurnForAngle(90,.3));
	AddSequential(new MoveForTime(5,.5));
	AddSequential(new PointTurnForAngle(90,.3));
	AddSequential(new MoveForTime(5,.5));
	AddSequential(new PointTurnForAngle(90,.3));
	AddSequential(new MoveForTime(5,.5));
}
