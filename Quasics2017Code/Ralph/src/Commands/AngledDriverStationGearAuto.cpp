#include "AngledDriverStationGearAuto.h"
#include "MoveForDistance.h"
#include "PointTurnForAngle.h"
#include "GearAuto.h"
#include "PixyAutoFineTuning.h"
#include "AutomaticLighting.h"
#include "SetLightColor.h"
#include "SetLightDynamics.h"
#include "../Subsystems/Arduino.h"
#include "../Robot.h"
AngledDriverStationGearAuto::AngledDriverStationGearAuto() {
	AddParallel(new AutomaticLighting);
	AddSequential(new MoveForDistance(-45,.5));
	AddSequential(new PointTurnForAngle(-35,.3));
	AddSequential(new MoveForDistance(-7,.25));
#ifdef Use_Pixy_Auto
	AddSequential(new PixyAutoFineTuning);
#endif
	AddSequential(new GearAuto(true));
}
