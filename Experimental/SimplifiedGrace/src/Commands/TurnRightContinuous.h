/*
 * TurnRightContinuous.h
 *
 *  Created on: Nov 3, 2017
 *      Author: Developer
 */

#ifndef SRC_COMMANDS_TURNRIGHTCONTINUOUS_H_
#define SRC_COMMANDS_TURNRIGHTCONTINUOUS_H_

#include "../CommandBase.h"

class TurnRightContinuous: public CommandBase {
public:
	TurnRightContinuous();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* SRC_COMMANDS_TURNRIGHTCONTINUOUS_H_ */
