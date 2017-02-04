#include "AngledDriverStationGearAuto.h"
//$$$
#include "MoveForDistance.h"
#include "PointTurnForAngle.h"
#include "GearAuto.h"

AngledDriverStationGearAuto::AngledDriverStationGearAuto() {
	AddSequential(new MoveForDistance(45,.5));
	AddSequential(new PointTurnForAngle(35,.3));
	AddSequential(new MoveForDistance(7,.25));
	AddSequential(new GearAuto(.5));
}
