/*
 * AutoModeCrossTheLineCommand.h
 *
 *  Created on: Mar 24, 2018
 *      Author: Developer
 */

#ifndef SRC_COMMANDS_AUTOMODECROSSTHELINECOMMAND_H_
#define SRC_COMMANDS_AUTOMODECROSSTHELINECOMMAND_H_

#include <frc/commands/ConditionalCommand.h>

class AutoModeCrossTheLineCommand: public frc::ConditionalCommand {
public:
	AutoModeCrossTheLineCommand();

	bool Condition() override;
};

#endif /* SRC_COMMANDS_AUTOMODECROSSTHELINECOMMAND_H_ */
