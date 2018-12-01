/*
 * SimpleMoveSequence.cpp
 *
 *  Created on: Dec 1, 2018
 *      Author: Developer
 */

#include <Commands/SimpleMoveSequence.h>
#include "Commands/TimedMove.h"

SimpleMoveSequence::SimpleMoveSequence() {
	// TODO Auto-generated constructor stub
	AddSequential(new TimedMove(.5, .2));
	AddSequential(new TimedMove(.5, -.2));
}

