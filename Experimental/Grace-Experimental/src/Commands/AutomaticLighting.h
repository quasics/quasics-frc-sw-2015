/*
 * AutomaticLighting.h
 *
 *  Created on: Nov 9, 2017
 *      Author: healym
 */

#ifndef SRC_COMMANDS_AUTOMATICLIGHTING_H_
#define SRC_COMMANDS_AUTOMATICLIGHTING_H_

#include <Commands/Command.h>

class AutomaticLighting: public frc::Command {
public:
	AutomaticLighting();

	void Execute() override;
	bool IsFinished() override;
};

#endif /* SRC_COMMANDS_AUTOMATICLIGHTING_H_ */
