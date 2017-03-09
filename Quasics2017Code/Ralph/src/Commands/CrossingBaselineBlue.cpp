#include "CrossingBaselineBlue.h"
#include "MoveForDistance.h"

CrossingBaselineBlue::CrossingBaselineBlue() {

	AddSequential(new MoveForDistance(96, .3));
}
