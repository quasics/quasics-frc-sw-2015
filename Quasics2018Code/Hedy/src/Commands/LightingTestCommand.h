/*
 * LightingTestCommand.h
 *
 *  Created on: Mar 17, 2018
 *      Author: healym
 */

#ifndef SRC_COMMANDS_LIGHTINGTESTCOMMAND_H_
#define SRC_COMMANDS_LIGHTINGTESTCOMMAND_H_

#include <Commands/Command.h>

class LightingTestCommand: public frc::Command {
public:
	LightingTestCommand();
	void Initialize() override;
	bool IsFinished() override;
};

#endif /* SRC_COMMANDS_LIGHTINGTESTCOMMAND_H_ */
