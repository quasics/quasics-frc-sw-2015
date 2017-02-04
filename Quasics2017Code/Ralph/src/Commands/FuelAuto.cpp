#include "FuelAuto.h"
#include "PointTurnForAngle.h"
#include "MoveForDistance.h"
#include "OutputAuto.h"
FuelAuto::FuelAuto() {

	AddSequential(new MoveForDistance(6,.5));
	AddSequential(new PointTurnForAngle(-45,.5));
	AddSequential(new MoveForDistance(-4, .5));
	//
	AddSequential(new OutputAuto(.5));
	AddSequential(new MoveForDistance(2,.5));
	AddSequential(new PointTurnForAngle(45, .5));
	AddSequential(new MoveForDistance(8, .5));

}
