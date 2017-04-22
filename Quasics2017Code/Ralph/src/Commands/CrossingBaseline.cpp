#include <Commands/CrossingBaseline.h>
#include "MoveForTime.h"

CrossingBaseline::CrossingBaseline() {

AddSequential(new MoveForTime(3, .25));

}
