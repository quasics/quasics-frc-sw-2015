/*
 * ExecuteTurn.h
 *
 *  Created on: Nov 4, 2017
 *      Author: Developer
 */

#ifndef SRC_COMMANDS_EXECUTETURN_H_
#define SRC_COMMANDS_EXECUTETURN_H_

#include "../CommandBase.h"

class ExecuteTurn: public CommandBase {
private:
	float degrees;
	double powerLevel;
	float targetHeading;

public:
	ExecuteTurn(float degrees, double powerLevel);

	// From Command interface
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};

#endif /* SRC_COMMANDS_EXECUTETURN_H_ */
