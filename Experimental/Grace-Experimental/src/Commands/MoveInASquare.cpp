/*
 * MoveInASquare.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: Developer
 */

#include "MoveInASquare.h"
#include "MoveForward.h"
#include "ExecuteTurn.h"

MoveInASquare::MoveInASquare(double powerLevel, double seconds) {
	// Go in a simple square.
	const double RIGHT_ANGLE_DEGREES = 85;		// Yes, this should be 90.  But the NavX appears to be overcalculating.
	for (int i = 0; i < 4; ++i) {
		AddSequential(new MoveForward(powerLevel, seconds));
		AddSequential(new ExecuteTurn(RIGHT_ANGLE_DEGREES, powerLevel));
	}
}
