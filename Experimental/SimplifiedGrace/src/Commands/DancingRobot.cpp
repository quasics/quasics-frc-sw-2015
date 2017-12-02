#include "TurnRightInput.h"
#include "DancingRobot.h"

DancingRobot::DancingRobot() {
	AddSequential(new TurnRightInput(90));
}
