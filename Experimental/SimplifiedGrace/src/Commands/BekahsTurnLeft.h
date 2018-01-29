/*
 * BekahsTurnLeft.h
 *
 *  Created on: Nov 16, 2017
 *      Author: rad100
 */

#ifndef SRC_COMMANDS_BEKAHSTURNLEFT_H_
#define SRC_COMMANDS_BEKAHSTURNLEFT_H_

#include <Commands/Command.h>

class BekahsTurnLeft: public frc::Command {
public:
	BekahsTurnLeft();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* SRC_COMMANDS_BEKAHSTURNLEFT_H_ */
