/*
 * AutomaticLightingCommand.h
 *
 *  Created on: Feb 18, 2017
 *      Author: healym
 */

#ifndef SRC_COMMANDS_AUTOMATICLIGHTINGCOMMAND_H_
#define SRC_COMMANDS_AUTOMATICLIGHTINGCOMMAND_H_

#include <Commands/Command.h>

class AutomaticLightingCommand: public frc::Command {
public:
	AutomaticLightingCommand();
	virtual ~AutomaticLightingCommand();

	void Execute() override;
	bool IsFinished() override;
};

#endif /* SRC_COMMANDS_AUTOMATICLIGHTINGCOMMAND_H_ */
