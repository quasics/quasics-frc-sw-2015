/*
 * TurnToNorth.h
 *
 *  Created on: Dec 15, 2018
 *      Author: Developer
 */

#ifndef SRC_COMMANDS_TURNTONORTH_H_
#define SRC_COMMANDS_TURNTONORTH_H_

#include <Commands/Command.h>

class TurnToNorth: public frc::Command {
public:
	TurnToNorth();

	void Initialize() override;
	void End() override;
	void Interrupted() override;
	bool IsFinished() override;
};

#endif /* SRC_COMMANDS_TURNTONORTH_H_ */
