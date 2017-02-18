/*
 * AutomaticLightingCommand.cpp
 *
 *  Created on: Feb 18, 2017
 *      Author: healym
 */

#include "AutomaticLightingCommand.h"
#include "../Robot.h"

AutomaticLightingCommand::AutomaticLightingCommand()
	: Command("Automatic Lighting")
{
	Requires(Robot::lighting.get());
}

AutomaticLightingCommand::~AutomaticLightingCommand() {
	// TODO Auto-generated destructor stub
}

void AutomaticLightingCommand::Execute() {
	Robot::lighting->updateState();
}

bool AutomaticLightingCommand::IsFinished() {
	return false;
}
