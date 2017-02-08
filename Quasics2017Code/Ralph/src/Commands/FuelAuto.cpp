#include "FuelAuto.h"
#include "PointTurnForAngle.h"
#include "MoveForDistance.h"
#include "OutputAuto.h"

FuelAuto::FuelAuto() {
	AddSequential(new MoveForDistance(39,.5));
	AddSequential(new PointTurnForAngle(-45,.5));
	AddSequential(new MoveForDistance(-4, .5));
	//
	AddSequential(new OutputAuto(.5));
	AddSequential(new MoveForDistance(10,.5));
	AddSequential(new PointTurnForAngle(45, .5));
	AddSequential(new MoveForDistance(20, .5));
}
