/*
 * SimpleMoveSequence.cpp
 *
 *  Created on: Dec 1, 2018
 *      Author: Developer
 */

#include "Commands/SimpleMoveSequence.h"
#include "Commands/TimedMove.h"
#include "Commands/LeftTurn.h"


SimpleMoveSequence::SimpleMoveSequence() {
	// I Made a Square!!!
	AddSequential(new TimedMove(1.00, .2, .3));
	AddSequential(new LeftTurn(.5, .27));
	AddSequential(new TimedMove(1.00, .2, .3));
	AddSequential(new LeftTurn(.5, .27));
	AddSequential(new TimedMove(1.00, .2, .3));
	AddSequential(new LeftTurn(.5, .27));
	AddSequential(new TimedMove(1.00, .2, .3));

}

