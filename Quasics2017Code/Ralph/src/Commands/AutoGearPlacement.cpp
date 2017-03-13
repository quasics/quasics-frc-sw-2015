#include "AutoGearPlacement.h"
#include "MoveForDistance.h"
#include "GearAuto.h"
#include "PixyAutoFineTuning.h"
#include "../Robot.h"

AutoGearPlacement::AutoGearPlacement() {
	AddSequential(new MoveForDistance(-52,.5));
#ifdef Use_Pixy_Auto
	AddSequential(new PixyAutoFineTuning);
#endif
	AddSequential(new GearAuto(true));
}
