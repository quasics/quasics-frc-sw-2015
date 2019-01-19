/*
 * AutoModeCrossTheLineCommand.cpp
 *
 *  Created on: Mar 24, 2018
 *      Author: Developer
 */

#include <Commands/AutoModeCrossTheLineCommand.h>
#include "Commands/AutoCrossTheLineFromMiddleCommand.h"
#include "Commands/AutoCrossTheLineFromSideCommand.h"
#include "Robot.h"

AutoModeCrossTheLineCommand::AutoModeCrossTheLineCommand()
: ConditionalCommand(new AutoCrossTheLineFromMiddleCommand, new AutoCrossTheLineFromSideCommand) {
	// TODO Auto-generated constructor stub

}

// Returns true iff we're starting in the middle (according to the console)
bool AutoModeCrossTheLineCommand::Condition(){
	Robot::RobotStartingPosition startingPosition = Robot::getStartingPosition();

	return (startingPosition == Robot::eStartingInMiddle);
}
