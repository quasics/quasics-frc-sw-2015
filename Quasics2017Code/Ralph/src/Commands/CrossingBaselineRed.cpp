#include "CrossingBaselineRed.h"
#include "MoveForDistance.h"

CrossingBaselineRed::CrossingBaselineRed() {

AddSequential(new MoveForDistance(96, .3));

}
