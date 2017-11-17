/*
 * BekahsBox.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: rad100
 */

#include "BekahsBox.h"

#include "BekahsTurnLeft.h"
#include "MoveForward.h"


BekahsBox::BekahsBox() {
	// First side of the box
	AddSequential(new MoveForward(.25, 1));
	AddSequential(new BekahsTurnLeft);

	// Second side of the box
	AddSequential(new MoveForward(.25, 1));
	AddSequential(new BekahsTurnLeft);

	// Third side of the box
	AddSequential(new MoveForward(.25, 1));
	AddSequential(new BekahsTurnLeft);

	// Fourth side of the box
	AddSequential(new MoveForward(.25, 1));
	AddSequential(new BekahsTurnLeft);
}
