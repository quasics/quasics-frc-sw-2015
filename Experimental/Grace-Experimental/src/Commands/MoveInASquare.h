/*
 * MoveInASquare.h
 *
 *  Created on: Nov 9, 2017
 *      Author: Developer
 */

#ifndef SRC_COMMANDS_MOVEINASQUARE_H_
#define SRC_COMMANDS_MOVEINASQUARE_H_

#include <Commands/CommandGroup.h>

class MoveInASquare: public frc::CommandGroup {
public:
	// A very small, fairly slow square (for test purposes).
	MoveInASquare() : MoveInASquare(.25, 1) {}

	MoveInASquare(double powerLevel, double seconds);
};

#endif /* SRC_COMMANDS_MOVEINASQUARE_H_ */
