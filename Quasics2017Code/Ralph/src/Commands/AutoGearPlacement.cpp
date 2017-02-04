#include "AutoGearPlacement.h"
#include "MoveForDistance.h"
#include "GearAuto.h"

AutoGearPlacement::AutoGearPlacement() {
	AddSequential(new MoveForDistance(-52,.5));
	AddSequential(new GearAuto(true));
}
