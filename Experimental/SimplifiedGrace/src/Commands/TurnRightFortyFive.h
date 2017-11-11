/*
 * TurnRightFortyFive.h
 *
 *  Created on: Nov 10, 2017
 *      Author: sth101
 */

#ifndef SRC_COMMANDS_TURNRIGHTFORTYFIVE_H_
#define SRC_COMMANDS_TURNRIGHTFORTYFIVE_H_

#include <Commands/Command.h>

class TurnRightFortyFive : public frc::Command {
public:
	TurnRightFortyFive();
	virtual ~TurnRightFortyFive();

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};

#endif /* SRC_COMMANDS_TURNRIGHTFORTYFIVE_H_ */

