#include "FuelAuto.h"
#include "PointTurnForAngle.h"
#include "MoveForTime.h"
#include "OutputAuto.h"
FuelAuto::FuelAuto() {

	AddSequential(new MoveForTime(6,.5));
	AddSequential(new PointTurnForAngle(45,.5));
	AddSequential(new MoveForTime(4, .5));
	//
	AddSequential(new OutputAuto(.5));
	AddSequential(new MoveForTime(-2,.5));
	AddSequential(new PointTurnForAngle(45, .5));
	AddSequential(new MoveForTime(-8, .5));

}
