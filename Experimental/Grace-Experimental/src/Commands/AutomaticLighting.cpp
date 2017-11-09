/*
 * AutomaticLighting.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: healym
 */

#include "AutomaticLighting.h"
#include "../Robot.h"

AutomaticLighting::AutomaticLighting()
{
	Requires(Robot::lighting.get());
}

void AutomaticLighting::Execute() {
	Robot::lighting->updateState();
}

bool AutomaticLighting::IsFinished() {
	return false;
}

