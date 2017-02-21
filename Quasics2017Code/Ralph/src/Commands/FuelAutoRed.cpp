#include "FuelAutoRed.h"
#include "PointTurnForAngle.h"
#include "MoveForDistance.h"
#include "OutputAuto.h"

// CODE_REVIEW(mjh): Document what this class actually does.
//For Red Alliance
FuelAutoRed::FuelAutoRed() {
	AddSequential(new MoveForDistance(39,.5));
	AddSequential(new PointTurnForAngle(45,.5));
	AddSequential(new MoveForDistance(-4, .5));
	AddSequential(new OutputAuto(.5));
	AddSequential(new MoveForDistance(10,.5));
	AddSequential(new PointTurnForAngle(-45, .5));
	AddSequential(new MoveForDistance(20, .5));
}
