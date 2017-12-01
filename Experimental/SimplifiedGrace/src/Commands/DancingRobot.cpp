#include "DancingRobot.h"
#include "TurnLeftInput.h"

DancingRobot::DancingRobot() {
	AddSequential(new TurnLeftInput(30));
}
