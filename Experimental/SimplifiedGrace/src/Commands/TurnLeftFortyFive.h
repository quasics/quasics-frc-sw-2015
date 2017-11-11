/*
 * TurnLeftFortyFive.h
 *
 *  Created on: Nov 10, 2017
 *      Author: sth101
 */

#ifndef SRC_COMMANDS_TURNLEFTFORTYFIVE_H_
#define SRC_COMMANDS_TURNLEFTFORTYFIVE_H_

#include <Commands/Command.h>

class TurnLeftFortyFive : public frc::Command {
public:
	TurnLeftFortyFive();
	virtual ~TurnLeftFortyFive();

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};

#endif /* SRC_COMMANDS_TURNLEFTFORTYFIVE_H_ */
