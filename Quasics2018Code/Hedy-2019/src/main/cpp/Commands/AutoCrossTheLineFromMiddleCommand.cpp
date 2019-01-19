/*
 * AutoCrossTheLineFromMiddle.cpp
 *
 *  Created on: Mar 24, 2018
 *      Author: Developer
 */

#include <Commands/AutoCrossTheLineFromMiddleCommand.h>
#include "Primitives/MoveForDistance.h"
#include <iostream>

AutoCrossTheLineFromMiddleCommand::AutoCrossTheLineFromMiddleCommand() {
	std::cout << "Crossing the line from the middle" << std::endl;
	// TODO Auto-generated constructor stub
	AddSequential(new MoveForDistance(60, -.4));		// Move 5' forward, to cross the line.


}

