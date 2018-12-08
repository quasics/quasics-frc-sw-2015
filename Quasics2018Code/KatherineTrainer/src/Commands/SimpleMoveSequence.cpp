/*
 * SimpleMoveSequence.cpp
 *
 *  Created on: Dec 1, 2018
 *      Author: Developer
 */

#include <Commands/SimpleMoveSequence.h>
#include "Commands/TimedMove.h"
#include "LeftTurn.h"

SimpleMoveSequence::SimpleMoveSequence() {
	// TODO Auto-generated constructor stub
	AddSequential(new TimedMove(1.0, .2));
	AddSequential(new LeftTurn(.5, .2));
	AddSequential(new TimedMove(1.0, .2));
	AddSequential(new LeftTurn(.5, .2));
	AddSequential(new TimedMove(1.0, .2));
}

